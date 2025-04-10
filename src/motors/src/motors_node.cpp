#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_driver.hpp"
#include "timer.hpp"

class Motors : public rclcpp::Node {
   public:
    std::vector<float> kp_, kd_;
    int can0_startID_, can0_endID_, can1_startID_, can1_endID_;
    Motors() : Node("motors_node") {
        kp_.resize(6);
        kd_.resize(6);

        this->declare_parameter<std::vector<float>>(
            "kp", std::vector<float>{150.0, 120.0, 150.0, 75.0, 75.0, 75.0});
        this->declare_parameter<std::vector<float>>("kd", std::vector<float>{4.0, 4.0, 2.0, 3.0, 2.0, 2.0});
        this->declare_parameter<int>("can0_startID", 0);
        this->declare_parameter<int>("can0_endID", 0);
        this->declare_parameter<int>("can1_startID", 0);
        this->declare_parameter<int>("can1_endID", 0);

        std::vector<double> tmp;
        this->get_parameter("kp", tmp);
        std::transform(tmp.begin(), tmp.end(), kp_.begin(),
                       [](double val) { return static_cast<float>(val); });
        this->get_parameter("kd", tmp);
        std::transform(tmp.begin(), tmp.end(), kd_.begin(),
                       [](double val) { return static_cast<float>(val); });
        this->get_parameter("can0_startID", can0_startID_);
        this->get_parameter("can0_endID", can0_endID_);
        this->get_parameter("can1_startID", can1_startID_);
        this->get_parameter("can1_endID", can1_endID_);

        RCLCPP_INFO(this->get_logger(), "kp: %f, %f, %f, %f, %f, %f", kp_[0], kp_[1], kp_[2], kp_[3], kp_[4],
                    kp_[5]);
        RCLCPP_INFO(this->get_logger(), "kd: %f, %f, %f, %f, %f, %f", kd_[0], kd_[1], kd_[2], kd_[3], kd_[4],
                    kd_[5]);
        RCLCPP_INFO(this->get_logger(), "can0_startID: %d", can0_startID_);
        RCLCPP_INFO(this->get_logger(), "can0_endID: %d", can0_endID_);
        RCLCPP_INFO(this->get_logger(), "can1_startID: %d", can1_startID_);
        RCLCPP_INFO(this->get_logger(), "can1_endID: %d", can1_endID_);

        for (int i = can0_startID_; i <= can0_endID_; i++) {
            left_motors[i - can0_startID_] = MotorDriver::MotorCreate(i, "can0", "DM");
            left_motors[i - can0_startID_]->MotorInit();
            left_motors[i - can0_startID_]->refresh_motor_status();
            // Timer::ThreadSleepFor(100);
            // left_motors[i - can0_startID_]->MotorSetZero(); //标零用
        }
        for (int i = can1_startID_; i <= can1_endID_; i++) {
            right_motors[i - can1_startID_] = MotorDriver::MotorCreate(i, "can1", "DM");
            right_motors[i - can1_startID_]->MotorInit();
            right_motors[i - can1_startID_]->refresh_motor_status();
            // Timer::ThreadSleepFor(100);
            // left_motors[i - can1_startID_]->MotorSetZero();
        }
        Timer::ThreadSleepFor(5);

        left_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_left", 1, std::bind(&Motors::subs_left_callback, this, std::placeholders::_1));
        right_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_right", 1, std::bind(&Motors::subs_right_callback, this, std::placeholders::_1));
        left_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_left", 1);
        right_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_right", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                         std::bind(&Motors::publish_joint_states, this));
    }
    ~Motors() {
        for (int i = 0; i < 6; i++) {
            left_motors[i]->MotorDeInit();
            right_motors[i]->MotorDeInit();
        }
    }

   private:
    std::shared_ptr<MotorDriver> left_motors[6];
    std::shared_ptr<MotorDriver> right_motors[6];

    void subs_left_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        for (int i = 0; i < 6; i++) {
            left_motors[i]->MotorMitModeCmd(msg->position[i], msg->velocity[i], kp_[i], kd_[i],
                                            msg->effort[i]);
        }
    }

    void subs_right_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        for (int i = 0; i < 6; i++) {
            right_motors[i]->MotorMitModeCmd(msg->position[i], msg->velocity[i], kp_[i], kd_[i],
                                             msg->effort[i]);
        }
    }

    void publish_joint_states() {
        for (int i = 0; i < 6; i++) {
            left_motors[i]->refresh_motor_status();
            right_motors[i]->refresh_motor_status();
        }
        auto left_message = sensor_msgs::msg::JointState();
        left_message.header.stamp = this->now();
        left_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        left_message.position = {left_motors[0]->get_motor_pos(), left_motors[1]->get_motor_pos(),
                                 left_motors[2]->get_motor_pos(), left_motors[3]->get_motor_pos(),
                                 left_motors[4]->get_motor_pos(), left_motors[5]->get_motor_pos()};
        left_message.velocity = {left_motors[0]->get_motor_spd(), left_motors[1]->get_motor_spd(),
                                 left_motors[2]->get_motor_spd(), left_motors[3]->get_motor_spd(),
                                 left_motors[4]->get_motor_spd(), left_motors[5]->get_motor_spd()};
        left_message.effort = {left_motors[0]->get_motor_current(), left_motors[1]->get_motor_current(),
                               left_motors[2]->get_motor_current(), left_motors[3]->get_motor_current(),
                               left_motors[4]->get_motor_current(), left_motors[5]->get_motor_current()};

        left_publisher_->publish(left_message);
        // RCLCPP_INFO(this->get_logger(), "Left Published JointState");

        auto right_message = sensor_msgs::msg::JointState();
        right_message.header.stamp = this->now();
        right_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        right_message.position = {right_motors[0]->get_motor_pos(), right_motors[1]->get_motor_pos(),
                                  right_motors[2]->get_motor_pos(), right_motors[3]->get_motor_pos(),
                                  right_motors[4]->get_motor_pos(), right_motors[5]->get_motor_pos()};
        right_message.velocity = {right_motors[0]->get_motor_spd(), right_motors[1]->get_motor_spd(),
                                  right_motors[2]->get_motor_spd(), right_motors[3]->get_motor_spd(),
                                  right_motors[4]->get_motor_spd(), right_motors[5]->get_motor_spd()};
        right_message.effort = {right_motors[0]->get_motor_current(), right_motors[1]->get_motor_current(),
                                right_motors[2]->get_motor_current(), right_motors[3]->get_motor_current(),
                                right_motors[4]->get_motor_current(), right_motors[5]->get_motor_current()};

        right_publisher_->publish(right_message);
        // RCLCPP_INFO(this->get_logger(), "Right Published JointState");
    }
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_publisher_, right_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_subscription_, right_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Motors>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}