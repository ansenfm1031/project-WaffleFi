#include "rosworker.h"
#include <QThread>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cmath>
#include "wifi_interface/msg/wifi_fused.hpp"


static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

void RosWorker::requestNavigateTo(double x, double y, double yaw_rad)
{
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_x_ = x;
    goal_y_ = y;
    goal_yaw_ = yaw_rad;
    goal_pending_ = true;
}

void RosWorker::run()
{
    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp::Node>("qt_visualizer_node", opts);
    exec_.add_node(node_);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    pose_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            try {
                auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                //const double yaw = tf2::getYaw(tf.transform.rotation);
                const auto &q = tf.transform.rotation;
                const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                const double yaw = std::atan2(siny_cosp, cosy_cosp);


                last_x_ = x; last_y_ = y; last_yaw_ = yaw;
                have_pose_ = true;

                emit robotPose(x, y, yaw);
            } catch (...) {
                // ignore
            }
        }
        );

    // /wifi/fused
    emit statusChanged("Subscribing /wifi/fused ...");
    // fused_sub_ = node_->create_subscription<wifi_interface::msg::WifiFused>(
    //     "/wifi/fused", rclcpp::QoS(10),
    //     [this](wifi_interface::msg::WifiFused::SharedPtr msg)
    //     {
    //         if (msg->ssid.empty()) return;

    //         // msg->x, msg->y : meter in map frame
    //         // msg->rssi      : float
    //         const int rssi_i = (int)std::lround(msg->rssi);

    //         emit fusedSample(msg->x, msg->y, QString::fromStdString(msg->ssid), rssi_i);
    //     }
    //     );
    fused_sub_ = node_->create_subscription<wifi_interface::msg::WifiFused>(
        "/wifi/fused", rclcpp::QoS(10),
        [this](wifi_interface::msg::WifiFused::SharedPtr msg)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "[Qt] RX /wifi/fused: ssid=%s x=%.2f y=%.2f rssi=%.1f",
                msg->ssid.c_str(), msg->x, msg->y, msg->rssi
                );

            if (msg->ssid.empty()) return;

            const int rssi_i = (int)std::lround(msg->rssi);
            emit fusedSample(msg->x, msg->y,
                             QString::fromStdString(msg->ssid),
                             rssi_i);
        }
        );


    // Nav2 action client
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
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

        // pending goal 처리
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
                goal_msg.pose.pose.position.z = 0.0;
                goal_msg.pose.pose.orientation = yawToQuat(gyaw);

                emit goalStatus(QString("Sending goal: (%1, %2, yaw=%3)")
                                    .arg(gx,0,'f',2).arg(gy,0,'f',2).arg(gyaw,0,'f',2));

                rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

                opts.goal_response_callback =
                    [this](std::shared_ptr<GoalHandleNav> gh)
                {
                    if (!gh) {
                        emit goalStatus("Goal rejected.");
                        return;
                    }
                    active_goal_ = gh;
                    emit goalStatus("Goal accepted.");
                };

                opts.feedback_callback =
                    [this](std::shared_ptr<GoalHandleNav>,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
                {
                    (void)feedback;
                };

                opts.result_callback =
                    [this](const GoalHandleNav::WrappedResult &result)
                {
                    switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        emit goalStatus("Goal SUCCEEDED.");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        emit goalStatus("Goal ABORTED.");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        emit goalStatus("Goal CANCELED.");
                        break;
                    default:
                        emit goalStatus("Goal result unknown.");
                        break;
                    }
                    active_goal_.reset();
                };

                nav_client_->async_send_goal(goal_msg, opts);
            }
        }

        QThread::msleep(20);
    }
}
