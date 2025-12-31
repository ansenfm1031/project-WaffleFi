#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "wifi_interface/msg/wifi_fused.hpp"


class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);
    void requestNavigateTo(double x, double y, double yaw_rad);

signals:
    void statusChanged(const QString &msg);

    // Robot pose (meter in map frame)
    void robotPose(double x, double y, double yaw);

    // (선택) test모드 dummy RSSI가 있으면 사용
    void sample(double x, double y, double yaw, float rssi);

    // UI goal status
    void goalStatus(const QString &msg);

    // Live heatmap: from /wifi/fused (meter)
    void fusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    // (옵션) gridPose를 쓴다면 유지
    void gridPose(double gx, double gy);

    // RL state arrived (현재는 단순 알림)
    void rlStateArrived();

protected:
    void run() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // Nav2 action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<GoalHandleNav> active_goal_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    bool have_pose_ = false;
    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;

    // ROS node/executor
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    // /wifi/fused subscription
    rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr fused_sub_;

    // (옵션) grid topic subscription을 따로 쓰면 유지
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_grid_;

    // pending goal storage (thread-safe)
    std::mutex goal_mtx_;
    bool goal_pending_ = false;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_yaw_ = 0.0;
};

#endif // ROSWORKER_H
