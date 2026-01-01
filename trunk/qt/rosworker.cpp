#include "rosworker.h"

#include <QThread>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "wifi_interface/msg/wifi_fused.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

geometry_msgs::msg::Quaternion RosWorker::yawToQuat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

void RosWorker::requestNavigateTo(double x, double y, double yaw_rad)
{
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_x_ = x;
    goal_y_ = y;
    goal_yaw_ = yaw_rad;
    goal_pending_ = true;
}

void RosWorker::requestHeatmap(const QString& session_id,
                               const QString& ssid,
                               bool thr_enable,
                               int thr_rssi,
                               uint32_t limit,
                               uint32_t offset)
{
    std::lock_guard<std::mutex> lk(heat_mtx_);
    heat_req_.pending = true;
    heat_req_.session_id = session_id;
    heat_req_.ssid = ssid;
    heat_req_.thr_enable = thr_enable;
    heat_req_.thr_rssi = thr_rssi;
    heat_req_.limit = limit;
    heat_req_.offset = offset;
}

void RosWorker::run()
{
    // rclcpp init (이미 init 되어있어도 OK)
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp::Node>("qt_visualizer_node", opts);
    exec_.add_node(node_);

    // TF
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    // TF pose timer (robot pose main source)
    pose_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            try {
                auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                const double yaw = tf2::getYaw(tf.transform.rotation);

                last_x_ = x; last_y_ = y; last_yaw_ = yaw;
                have_pose_ = true;

                have_pose_tf_ = true;
                last_tf_stamp_ = node_->now();

                emit robotPose(x, y, yaw);
            } catch (...) {
                // TF 없으면 AMCL fallback이 커버
            }
        }
        );

    // /wifi/fused = live heatmap source
    emit statusChanged("Subscribing /wifi/fused (live heatmap) ...");
    fused_sub_ = node_->create_subscription<wifi_interface::msg::WifiFused>(
        "/wifi/fused", rclcpp::QoS(10),
        [this](wifi_interface::msg::WifiFused::SharedPtr msg)
        {
            if (!msg) return;
            if (msg->ssid.empty()) return;

            const int rssi_i = (int)std::lround(msg->rssi);
            emit fusedSample(msg->x, msg->y,
                             QString::fromStdString(msg->ssid),
                             rssi_i);
        }
        );

    // /amcl_pose = fallback robot pose source (TF stale일 때만)
    emit statusChanged("Subscribing /amcl_pose (TF fallback) ...");
    amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            if (!msg) return;

            const double x = msg->pose.pose.position.x;
            const double y = msg->pose.pose.position.y;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
                );
            double roll=0, pitch=0, yaw=0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // AMCL 안정도 gating (수렴 전엔 무시)
            const auto &C = msg->pose.covariance;
            const double var_x   = C[0];
            const double var_y   = C[7];
            const double var_yaw = C[35];
            const bool stable = (var_x < 0.25) && (var_y < 0.25) && (var_yaw < 0.5);
            if (!stable) return;

            // TF가 최근이면 AMCL로 덮어쓰지 않음
            bool tf_recent = false;
            if (have_pose_tf_) {
                const auto now = node_->now();
                const double dt = (now - last_tf_stamp_).seconds();
                tf_recent = (dt >= 0.0 && dt < 0.5); // 0.5초 이내면 TF 정상
            }
            if (tf_recent) return;

            last_x_ = x; last_y_ = y; last_yaw_ = yaw;
            have_pose_ = true;
            emit robotPose(x, y, yaw);
        }
        );

    // Service client: /db/get_heatmap (past heatmap)
    heatmap_client_ = node_->create_client<wifi_interface::srv::GetHeatmap>("/db/get_heatmap");
    emit statusChanged("Heatmap service client ready: /db/get_heatmap");

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    emit statusChanged("Waiting Nav2 action server: navigate_to_pose ...");
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
            emit statusChanged("Nav2 action server connected.");
            break;
        }
    }

    while (rclcpp::ok() && !isInterruptionRequested()) {
        exec_.spin_some();

        // (1) pending heatmap request 처리 (서비스 호출)
        HeatReq hreq;
        {
            std::lock_guard<std::mutex> lk(heat_mtx_);
            if (heat_req_.pending) {
                hreq = heat_req_;
                heat_req_.pending = false;
            } else {
                hreq.pending = false;
            }
        }

        if (hreq.pending) {
            if (!heatmap_client_ || !heatmap_client_->service_is_ready()) {
                emit statusChanged("Service not ready: /db/get_heatmap");
                emit heatmapReplyArrived(false, "Service not ready",
                                         hreq.session_id, hreq.ssid,
                                         hreq.thr_enable, hreq.thr_rssi,
                                         {}, {}, {}, {}, {});
            } else {
                auto req = std::make_shared<wifi_interface::srv::GetHeatmap::Request>();
                req->session_id = hreq.session_id.toStdString();
                req->ssid       = hreq.ssid.toStdString();
                req->thr_enable = hreq.thr_enable;
                req->thr_rssi   = hreq.thr_rssi;
                req->limit      = hreq.limit;
                req->offset     = hreq.offset;

                emit statusChanged(QString("Calling /db/get_heatmap sid=%1 ssid=%2 thr=%3 limit=%4 offset=%5")
                                       .arg(hreq.session_id).arg(hreq.ssid).arg(hreq.thr_rssi)
                                       .arg(hreq.limit).arg(hreq.offset));

                auto cb =
                    [this, hreq](rclcpp::Client<wifi_interface::srv::GetHeatmap>::SharedFuture future)
                {
                    auto res = future.get();
                    if (!res) {
                        emit heatmapReplyArrived(false, "Null response",
                                                 hreq.session_id, hreq.ssid,
                                                 hreq.thr_enable, hreq.thr_rssi,
                                                 {}, {}, {}, {}, {});
                        return;
                    }

                    QVector<double> xs, ys;
                    QVector<int> rssis;
                    QVector<QString> ssids, stamps;

                    const size_t n = res->xs.size();
                    xs.reserve((int)n);
                    ys.reserve((int)n);
                    rssis.reserve((int)n);
                    ssids.reserve((int)n);
                    stamps.reserve((int)n);

                    for (size_t i=0; i<n; ++i) {
                        xs.push_back(res->xs[i]);
                        ys.push_back(res->ys[i]);
                        rssis.push_back((int)res->rssis[i]);
                        ssids.push_back(QString::fromStdString(res->ssids[i]));
                        stamps.push_back(QString::fromStdString(res->stamps[i]));
                    }

                    emit heatmapReplyArrived(res->ok,
                                             QString::fromStdString(res->message),
                                             hreq.session_id, hreq.ssid,
                                             hreq.thr_enable, hreq.thr_rssi,
                                             xs, ys, rssis, ssids, stamps);
                };

                heatmap_client_->async_send_request(req, cb);
            }
        }

        // (2) pending nav goal 처리
        bool do_send = false;
        double gx=0, gy=0, gyaw=0;
        {
            std::lock_guard<std::mutex> lk(goal_mtx_);
            if (goal_pending_) {
                gx = goal_x_;
                gy = goal_y_;
                gyaw = goal_yaw_;
                goal_pending_ = false;
                do_send = true;
            }
        }

        if (do_send) {
            if (!nav_client_ || !nav_client_->action_server_is_ready()) {
                emit goalStatus("Nav2 not ready: action server not available.");
            } else {
                NavigateToPose::Goal goal_msg;
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = node_->now();
                goal_msg.pose.pose.position.x = gx;
                goal_msg.pose.pose.position.y = gy;
                goal_msg.pose.pose.orientation = yawToQuat(gyaw);

                emit goalStatus(QString("Sending goal: (%1, %2, yaw=%3)")
                                    .arg(gx,0,'f',2).arg(gy,0,'f',2).arg(gyaw,0,'f',2));

                rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

                opts.goal_response_callback =
                    [this](std::shared_ptr<GoalHandleNav> gh)
                {
                    if (!gh) { emit goalStatus("Goal rejected."); return; }
                    active_goal_ = gh;
                    emit goalStatus("Goal accepted.");
                };

                opts.result_callback =
                    [this](const GoalHandleNav::WrappedResult &result)
                {
                    switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED: emit goalStatus("Goal SUCCEEDED."); break;
                    case rclcpp_action::ResultCode::ABORTED:   emit goalStatus("Goal ABORTED.");   break;
                    case rclcpp_action::ResultCode::CANCELED:  emit goalStatus("Goal CANCELED.");  break;
                    default:                                   emit goalStatus("Goal result unknown."); break;
                    }
                    active_goal_.reset();
                };

                nav_client_->async_send_goal(goal_msg, opts);
            }
        }

        QThread::msleep(20);
    }

    exec_.remove_node(node_);
    node_.reset();
}
