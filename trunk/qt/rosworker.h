#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <vector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "wifi_interface/msg/wifi_fused.hpp"
#include "wifi_interface/srv/get_heatmap.hpp"

class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject* parent=nullptr);

    // nav goal
    void requestNavigateTo(double x, double y, double yaw_rad);

    // past heatmap service request
    void requestHeatmap(const QString& session_id,
                        const QString& ssid,
                        bool thr_enable,
                        int thr_rssi,
                        uint32_t limit,
                        uint32_t offset);

signals:
    void statusChanged(const QString& msg);

    // robot pose in map frame
    void robotPose(double x, double y, double yaw);

    // live fused sample
    void fusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    // service reply for past heatmap
    void heatmapReplyArrived(bool ok,
                             const QString& message,
                             const QString& session_id,
                             const QString& ssid,
                             bool thr_enable,
                             int thr_rssi,
                             const QVector<double>& xs,
                             const QVector<double>& ys,
                             const QVector<int>& rssis,
                             const QVector<QString>& ssids,
                             const QVector<QString>& stamps);

    // nav goal status (optional)
    void goalStatus(const QString& msg);

protected:
    void run() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    static geometry_msgs::msg::Quaternion yawToQuat(double yaw);

    // ROS core
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Node::SharedPtr node_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    // pose cache
    double last_x_{0}, last_y_{0}, last_yaw_{0};
    bool have_pose_{false};

    // TF freshness for AMCL fallback
    bool have_pose_tf_{false};
    rclcpp::Time last_tf_stamp_;

    // subs
    rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr fused_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    // nav action
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<GoalHandleNav> active_goal_;

    std::mutex goal_mtx_;
    bool goal_pending_{false};
    double goal_x_{0}, goal_y_{0}, goal_yaw_{0};

    // heatmap service client
    rclcpp::Client<wifi_interface::srv::GetHeatmap>::SharedPtr heatmap_client_;

    struct HeatReq {
        bool pending{false};
        QString session_id;
        QString ssid;
        bool thr_enable{false};
        int thr_rssi{-70};
        uint32_t limit{50000};
        uint32_t offset{0};
    };
    std::mutex heat_mtx_;
    HeatReq heat_req_;
};

#endif
