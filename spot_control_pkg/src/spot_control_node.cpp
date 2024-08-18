#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include "Kinematics.h"
#include <iostream>
#include <iomanip>

using namespace std::chrono_literals;

#define A_H 170
#define A_Z  180/2
#define A_X  220/2
#define MAX_A_H 210
#define MIN_A_H 60

struct FootState {
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    double roll;
    bool contect;
    double force;
    double force_offset;
};

class KinematicCalculater : public rclcpp::Node
{
    public:
    KinematicCalculater() : Node("spot_control_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> (
            "set/joint_states", qos_profile );
        joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState> (
            "feedback/spot_joints", qos_profile, 
            std::bind(&KinematicCalculater::forward_kinematic, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            10ms, std::bind(&KinematicCalculater::calculator, this));
        point_publisher_FL = this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_point_FL", 10);
        input_point_publisher_FL = this->create_publisher<geometry_msgs::msg::PointStamped>("/input_point_FL", 10);
        point_publisher_FR = this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_point_FR", 10);
        input_point_publisher_FR = this->create_publisher<geometry_msgs::msg::PointStamped>("/input_point_FR", 10);
        point_publisher_BL = this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_point_BL", 10);
        input_point_publisher_BL = this->create_publisher<geometry_msgs::msg::PointStamped>("/input_point_BL", 10);
        point_publisher_BR = this->create_publisher<geometry_msgs::msg::PointStamped>("/desired_point_BR", 10);
        input_point_publisher_BR = this->create_publisher<geometry_msgs::msg::PointStamped>("/input_point_BR", 10);

        auto qos_profile2 = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        forceFL_subscribe_ = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
            "feedback/spot_forceFL", qos_profile2, 
            std::bind(&KinematicCalculater::forceFL_callback, this, std::placeholders::_1));
        forceFR_subscribe_ = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
            "feedback/spot_forceFR", qos_profile2, 
            std::bind(&KinematicCalculater::forceFR_callback, this, std::placeholders::_1));
        forceBL_subscribe_ = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
            "feedback/spot_forceBL", qos_profile2, 
            std::bind(&KinematicCalculater::forceBL_callback, this, std::placeholders::_1));
        forceBR_subscribe_ = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
            "feedback/spot_forceBR", qos_profile2, 
            std::bind(&KinematicCalculater::forceBR_callback, this, std::placeholders::_1));
        pose_subscribe_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_pose", 10, std::bind(&KinematicCalculater::pose_callback, this, std::placeholders::_1));

        joint_state_.name = {
            "fls", "fll", "flf",
            "frs", "frl", "frf",
            "bls", "bll", "blf",
            "brs", "brl", "brf"
        };
        joint_state_.position.resize(12);
        joint_state_.velocity.resize(12);
        // joint_state_.effort.resize(12);

        for (int i = 0; i <12; i++)
        joint_state_.velocity[i] = 1000;
    }

    private:
        
    void calculator() 
    {    
        double force_sum = FL_foot.force + FR_foot.force + BL_foot.force + BR_foot.force;
        
        double FL_x = + (FL_foot.force - force_sum);
        double FL_y = + (FL_foot.force - force_sum);
        double FR_x = + (FR_foot.force - force_sum);
        double FR_y = - (FR_foot.force - force_sum);
        double BL_x = - (BL_foot.force - force_sum);
        double BL_y = + (BL_foot.force - force_sum);
        double BR_x = - (BR_foot.force - force_sum);
        double BR_y = - (BR_foot.force - force_sum);

        double y_input = FL_y + FR_y + BL_y + BR_y;
        double x_input = FL_x + FR_x + BL_x + BR_x;

        std::cout << "xy input : " << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << x_input << " "
                                << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << y_input << std::endl;
        
        double p = 0.5;
        body_location[0] = body_location[0] - p * x_input;
        body_location[2] = body_location[2] - p * y_input;
        // std::cout << "force : " << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << FL_foot.force << " "
        //                         << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << FR_foot.force << " "
        //                         << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << BL_foot.force << " "
        //                         << std::setw(6) << std::setfill('0') << std::fixed << std::setprecision(4) << BR_foot.force << std::endl;
        
        kinematic.calcIK(returns, Resetfootpoint, body_pose, body_location);

        joint_state_.header.stamp = this->get_clock()->now();
        joint_state_.position[0] =   -returns[0][0];
        joint_state_.position[1] =   returns[0][1];// - 7.85 * pi/180;
        joint_state_.position[2] =   returns[0][2];// + 7.85 * pi/180;

        joint_state_.position[3] =   returns[1][0];
        joint_state_.position[4] =   returns[1][1];// - 7.85 * pi/180;
        joint_state_.position[5] =   returns[1][2];// + 7.85 * pi/180;
        
        joint_state_.position[6] =   -returns[2][0];
        joint_state_.position[7] =   returns[2][1];// - 7.85 * pi/180;
        joint_state_.position[8] =   returns[2][2];// + 7.85 * pi/180;
        
        joint_state_.position[9] =   returns[3][0];
        joint_state_.position[10] =  returns[3][1];// - 7.85 * pi/180;
        joint_state_.position[11] =  returns[3][2];// + 7.85 * pi/180;

        joint_publisher_->publish(joint_state_);
    }

    void forward_kinematic(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        double joint[4][4];
        for (int i = 0; i < 4; i++) {
            memcpy(&joint[i], &msg->position[i*3], 3*sizeof(double));
            joint[i][1] = joint[i][1];// + 7.85 * pi/180;
            joint[i][2] = joint[i][2];// - 7.85 * pi/180;
            joint[i][3] = 1;
        }
        double data[4][4];
        kinematic.bodyFK(data, joint, body_pose, body_location);

        auto message = geometry_msgs::msg::PointStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link";
        
        message.point.x = -data[0][0]/1000;
        message.point.y = -data[0][2]/1000;
        message.point.z = data[0][1]/1000;
        point_publisher_FL->publish(message);

        message.point.x = -Resetfootpoint[0][0]/1000;
        message.point.y = -Resetfootpoint[0][2]/1000;
        message.point.z = Resetfootpoint[0][1]/1000;
        input_point_publisher_FL->publish(message);

        message.point.x = -data[1][0]/1000;
        message.point.y = -data[1][2]/1000;
        message.point.z = data[1][1]/1000;
        point_publisher_FR->publish(message);
        
        message.point.x = -Resetfootpoint[1][0]/1000;
        message.point.y = -Resetfootpoint[1][2]/1000;
        message.point.z = Resetfootpoint[1][1]/1000;
        input_point_publisher_FR->publish(message);

        message.point.x = -data[2][0]/1000;
        message.point.y = -data[2][2]/1000;
        message.point.z = data[2][1]/1000;
        point_publisher_BL->publish(message);
        
        message.point.x = -Resetfootpoint[2][0]/1000;
        message.point.y = -Resetfootpoint[2][2]/1000;
        message.point.z = Resetfootpoint[2][1]/1000;
        input_point_publisher_BL->publish(message);

        message.point.x = -data[3][0]/1000;
        message.point.y = -data[3][2]/1000;
        message.point.z = data[3][1]/1000;
        point_publisher_BR->publish(message);
        
        message.point.x = -Resetfootpoint[3][0]/1000;
        message.point.y = -Resetfootpoint[3][2]/1000;
        message.point.z = Resetfootpoint[3][1]/1000;
        input_point_publisher_BR->publish(message);
    }

    void forceFL_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        FL_foot.force = sqrt(msg->wrench.force.x +msg->wrench.force.y + msg->wrench.force.z);
    }

    void forceFR_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        FR_foot.force = sqrt(msg->wrench.force.x +msg->wrench.force.y + msg->wrench.force.z);
    }

    void forceBL_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        BL_foot.force = sqrt(msg->wrench.force.x +msg->wrench.force.y + msg->wrench.force.z);
    }

    void forceBR_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        BR_foot.force = sqrt(msg->wrench.force.x +msg->wrench.force.y + msg->wrench.force.z);
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        double w = msg->orientation.w;
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;

        double roll, pitch, yaw;
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        pitch = asin(2 * (w * y - z * x));
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        body_pose[0] = roll;
        body_pose[2] = pitch;
        body_pose[1] = yaw;
        body_location[0] = msg->position.x;
        body_location[1] = msg->position.z;
        body_location[2] = msg->position.y;
    }


    double Resetfootpoint[4][4] =  {
        {A_X, -A_H, A_Z, 1}, 
        {A_X, -A_H, -A_Z, 1}, 
        {-A_X, -A_H, A_Z, 1}, 
        {-A_X, -A_H, -A_Z, 1}
    };
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_FL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr input_point_publisher_FL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_FR;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr input_point_publisher_FR;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_BL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr input_point_publisher_BL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_BR;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr input_point_publisher_BR;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
    Kinematic kinematic;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr forceFL_subscribe_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr forceFR_subscribe_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr forceBL_subscribe_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr forceBR_subscribe_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscribe_;
    double returns[4][3];
    double body_pose[3] = {0,0,0};
    double body_location[3] = {0,0,0};

    FootState FL_foot;
    FootState FR_foot;
    FootState BL_foot;
    FootState BR_foot;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicCalculater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}