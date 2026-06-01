#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <cmath>

class RacecarController {
public:
    RacecarController() : nh("~") {
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
        bend_sub = nh.subscribe("/empty_bend", 1, &RacecarController::bend_callback, this);
        imu_sub = nh.subscribe("/imu_data", 1, &RacecarController::imuCallback, this);

        // 初始化变量
        target_yaw = 0.0;
        is_turning = false;
        time_turn = 0;
        ready_to_turn = false; // 使用 bool 类型
    }

    void bend_callback(const std_msgs::String::ConstPtr& msg) {
        ready_to_turn = true; // 收到消息后设置为可转向
        ROS_INFO("start turning");
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 从四元数获取偏航角
        double roll, pitch, yaw;
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, 
                         msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // 检查四元数是否有效
        if (std::isnan(msg->orientation.x) || std::isnan(msg->orientation.y) ||
            std::isnan(msg->orientation.z) || std::isnan(msg->orientation.w)) {
            ROS_WARN("Received invalid quaternion values");
            return; // 退出回调函数
        }        

        // 输出实时 yaw 值
        ROS_INFO("Current Yaw: %f", yaw);

        // 检查是否需要转向
        if (!is_turning && time_turn == 0) {
            target_yaw = yaw + M_PI; // 设置目标偏航角为当前角度加180度
            is_turning = true;
        }
        ROS_INFO("target_yaw: %f", target_yaw);
        
        // 控制小车
        controlRacecar(yaw);
    }

    void controlRacecar(double current_yaw) {
        geometry_msgs::Twist cmd_vel;

        // 检查是否到达目标偏航角
        if (is_turning && fabs(current_yaw - target_yaw) < 0.2618) { // 15度转换为弧度
            // 停止小车
            cmd_vel.linear.x = 1500.0;
            cmd_vel.angular.z = 90;
            is_turning = false; // 重置转向状态
            time_turn = 1;
            ready_to_turn = false; // 重置转向标志位
        } else if (time_turn == 1) {
            // 向左移动
            cmd_vel.linear.x = 1500; // 根据需要调整速度
            cmd_vel.angular.z = 90; // 向左转
        } else {
            // 向左移动
            cmd_vel.linear.x = 1520; // 根据需要调整速度
            cmd_vel.angular.z = 150; // 向左转
        }

        // 只有在ready_to_turn为true时才发布命令
        if (ready_to_turn) {
            cmd_pub.publish(cmd_vel);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber bend_sub;
    double target_yaw;
    int8_t time_turn;
    bool is_turning;
    bool ready_to_turn; // 使用 bool 类型
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "racecar_controller");
    RacecarController racecar_controller;

    ros::spin();
    return 0;
}
