#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <mutex>

class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);
    void requestNavigateTo(double x, double y, double yaw_rad);

signals:
    void statusChanged(const QString &msg);
    void robotPose(double x, double y, double yaw);
    void sample(double x, double y, double yaw, float rssi);

    //ui 목표 지점 설정
    void goalStatus(const QString &msg);


protected:
    void run() override;

private:
    //ui 목표 지점 설정
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<GoalHandleNav> active_goal_;


    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    //pending goal storage (thread-safe)
    std::mutex goal_mtx_;
    bool goal_pending_ = false;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_yaw_ = 0.0;
};

#endif
