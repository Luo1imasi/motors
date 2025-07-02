#include <motors/srv/control_motor.hpp>
#include <motors/srv/read_motors.hpp>
#include <motors/srv/reset_motors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "close_chain_mapping.hpp"
#include "motor_driver.hpp"
#include "timer.hpp"

class MotorsNode : public rclcpp::Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MotorsNode() : Node("motors_node") {
        kp_.resize(12);
        kd_.resize(12);
        joint_default_angle_.resize(23);
        ankle_decouple_ = std::make_shared<Decouple>();

        this->declare_parameter<std::vector<float>>(
            "kp",
            std::vector<float>{100.0, 150.0, 100.0, 150.0, 40.0, 40.0, 100.0, 40.0, 40.0, 40.0, 40.0, 40.0});
        this->declare_parameter<std::vector<float>>(
            "kd", std::vector<float>{4.0, 5.0, 4.0, 5.0, 3.0, 3.0, 4.0, 3.0, 3.0, 3.0, 3.0, 3.0});
        this->declare_parameter<int>("can0_startID", 0);
        this->declare_parameter<int>("can0_endID", 0);
        this->declare_parameter<int>("can1_startID", 0);
        this->declare_parameter<int>("can1_endID", 0);
        this->declare_parameter<int>("can2_startID", 0);
        this->declare_parameter<int>("can2_endID", 0);
        this->declare_parameter<int>("can3_startID", 0);
        this->declare_parameter<int>("can3_endID", 0);
        this->declare_parameter<int>("can0_masterID_offset", 0);
        this->declare_parameter<int>("can1_masterID_offset", 0);
        this->declare_parameter<int>("can2_masterID_offset", 0);
        this->declare_parameter<int>("can3_masterID_offset", 0);
        this->declare_parameter<std::vector<float>>(
            "joint_default_angle",
            std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

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
        this->get_parameter("can2_startID", can2_startID_);
        this->get_parameter("can2_endID", can2_endID_);
        this->get_parameter("can3_startID", can3_startID_);
        this->get_parameter("can3_endID", can3_endID_);
        this->get_parameter("can0_masterID_offset", can0_masterID_offset_);
        this->get_parameter("can1_masterID_offset", can1_masterID_offset_);
        this->get_parameter("can2_masterID_offset", can2_masterID_offset_);
        this->get_parameter("can3_masterID_offset", can3_masterID_offset_);
        this->get_parameter("joint_default_angle", tmp);
        std::transform(tmp.begin(), tmp.end(), joint_default_angle_.begin(),
                       [](double val) { return static_cast<float>(val); });

        RCLCPP_INFO(this->get_logger(), "kp: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", kp_[0], kp_[1],
                    kp_[2], kp_[3], kp_[4], kp_[5], kp_[6], kp_[7], kp_[8], kp_[9], kp_[10], kp_[11]);
        RCLCPP_INFO(this->get_logger(), "kd: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", kd_[0], kd_[1],
                    kd_[2], kd_[3], kd_[4], kd_[5], kd_[6], kd_[7], kd_[8], kd_[9], kd_[10], kd_[11]);
        RCLCPP_INFO(this->get_logger(),
                    "joint_default_angle: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, "
                    "%f, %f, %f, %f, %f, %f, %f",
                    joint_default_angle_[0], joint_default_angle_[1], joint_default_angle_[2],
                    joint_default_angle_[3], joint_default_angle_[4], joint_default_angle_[5],
                    joint_default_angle_[6], joint_default_angle_[7], joint_default_angle_[8],
                    joint_default_angle_[9], joint_default_angle_[10], joint_default_angle_[11],
                    joint_default_angle_[12], joint_default_angle_[13], joint_default_angle_[14],
                    joint_default_angle_[15], joint_default_angle_[16], joint_default_angle_[17],
                    joint_default_angle_[18], joint_default_angle_[19], joint_default_angle_[20],
                    joint_default_angle_[21], joint_default_angle_[22]);
        RCLCPP_INFO(this->get_logger(), "can0_startID: %d", can0_startID_);
        RCLCPP_INFO(this->get_logger(), "can0_endID: %d", can0_endID_);
        RCLCPP_INFO(this->get_logger(), "can1_startID: %d", can1_startID_);
        RCLCPP_INFO(this->get_logger(), "can1_endID: %d", can1_endID_);
        RCLCPP_INFO(this->get_logger(), "can2_startID: %d", can2_startID_);
        RCLCPP_INFO(this->get_logger(), "can2_endID: %d", can2_endID_);
        RCLCPP_INFO(this->get_logger(), "can3_startID: %d", can3_startID_);
        RCLCPP_INFO(this->get_logger(), "can3_endID: %d", can3_endID_);
        RCLCPP_INFO(this->get_logger(), "can0_masterID_offset: %d", can0_masterID_offset_);
        RCLCPP_INFO(this->get_logger(), "can1_masterID_offset: %d", can1_masterID_offset_);
        RCLCPP_INFO(this->get_logger(), "can2_masterID_offset: %d", can2_masterID_offset_);
        RCLCPP_INFO(this->get_logger(), "can3_masterID_offset: %d", can3_masterID_offset_);

        left_ankle_motors_default_angle_ << joint_default_angle_[4], joint_default_angle_[5];
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(2);
        ankle_decouple_->getDecoupleQVT(left_ankle_motors_default_angle_, vel, tau, true);
        right_ankle_motors_default_angle_ << joint_default_angle_[10], joint_default_angle_[11];
        ankle_decouple_->getDecoupleQVT(right_ankle_motors_default_angle_, vel, tau, false);

        for (int i = can0_startID_; i <= can0_endID_; i++) {
            left_leg_motors[i - can0_startID_] =
                MotorDriver::MotorCreate(i, "can0", "DM", can0_masterID_offset_);
            left_leg_motors[i - can0_startID_]->MotorInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can1_startID_; i <= can1_endID_; i++) {
            right_leg_motors[i - can1_startID_] =
                MotorDriver::MotorCreate(i, "can1", "DM", can1_masterID_offset_);
            right_leg_motors[i - can1_startID_]->MotorInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can2_startID_; i <= can2_endID_; i++) {
            left_arm_motors[i - can2_startID_] =
                MotorDriver::MotorCreate(i, "can2", "DM", can2_masterID_offset_);
            left_arm_motors[i - can2_startID_]->MotorInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can3_startID_; i <= can3_endID_; i++) {
            right_arm_motors[i - can3_startID_] =
                MotorDriver::MotorCreate(i, "can3", "DM", can3_masterID_offset_);
            right_arm_motors[i - can3_startID_]->MotorInit();
            Timer::ThreadSleepFor(1);
        }
        Timer::ThreadSleepFor(500);
        // for (int i = can0_startID_; i <= can0_endID_; i++) {
        //     left_leg_motors[i - can0_startID_]->MotorSetZero();
        //     Timer::ThreadSleepFor(1);
        // }
        // for (int i = can1_startID_; i <= can1_endID_; i++) {
        //     right_leg_motors[i - can1_startID_]->MotorSetZero();
        //     Timer::ThreadSleepFor(1);
        // }
        // for (int i = can2_startID_; i <= can2_endID_; i++) {
        //     left_arm_motors[i - can2_startID_]->MotorSetZero();
        //     Timer::ThreadSleepFor(1);
        // }
        // for (int i = can3_startID_; i <= can3_endID_; i++) {
        //     right_arm_motors[i - can3_startID_]->MotorSetZero();
        //     Timer::ThreadSleepFor(1);
        // }
        left_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_left_leg", 1,
            std::bind(&MotorsNode::subs_left_leg_callback, this, std::placeholders::_1));
        right_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_right_leg", 1,
            std::bind(&MotorsNode::subs_right_leg_callback, this, std::placeholders::_1));
        left_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_left_arm", 1,
            std::bind(&MotorsNode::subs_left_arm_callback, this, std::placeholders::_1));
        right_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_right_arm", 1,
            std::bind(&MotorsNode::subs_right_arm_callback, this, std::placeholders::_1));
        left_leg_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_left_leg", 1);
        right_leg_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_right_leg", 1);
        left_arm_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_left_arm", 1);
        right_arm_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_right_arm", 1);
        control_motor_service_ = this->create_service<motors::srv::ControlMotor>(
            "control_motor",
            std::bind(&MotorsNode::control_motor, this, std::placeholders::_1, std::placeholders::_2));
        reset_motors_service_ = this->create_service<motors::srv::ResetMotors>(
            "reset_motors",
            std::bind(&MotorsNode::reset_motors, this, std::placeholders::_1, std::placeholders::_2));
        read_motors_service_ = this->create_service<motors::srv::ReadMotors>(
            "read_motors",
            std::bind(&MotorsNode::read_motors, this, std::placeholders::_1, std::placeholders::_2));
    }
    ~MotorsNode() {
        std::scoped_lock lock(left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_);
        for (int i = can0_startID_; i <= can0_endID_; i++) {
            left_leg_motors[i - can0_startID_]->MotorDeInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can1_startID_; i <= can1_endID_; i++) {
            right_leg_motors[i - can1_startID_]->MotorDeInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can2_startID_; i <= can2_endID_; i++) {
            left_arm_motors[i - can2_startID_]->MotorDeInit();
            Timer::ThreadSleepFor(1);
        }
        for (int i = can3_startID_; i <= can3_endID_; i++) {
            right_arm_motors[i - can3_startID_]->MotorDeInit();
            Timer::ThreadSleepFor(1);
        }
        RCLCPP_INFO(this->get_logger(), "Motors Deinitialized");
    }

   private:
    std::shared_ptr<MotorDriver> left_leg_motors[6], right_leg_motors[7], left_arm_motors[5],
        right_arm_motors[5];
    Eigen::VectorXd left_ankle_motors_default_angle_, right_ankle_motors_default_angle_;
    std::vector<float> kp_, kd_, joint_default_angle_;
    int can0_startID_, can0_endID_, can1_startID_, can1_endID_, can2_startID_, can2_endID_, can3_startID_,
        can3_endID_, can0_masterID_offset_, can1_masterID_offset_, can2_masterID_offset_,
        can3_masterID_offset_;
    std::shared_mutex left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_;
    rclcpp::Service<motors::srv::ControlMotor>::SharedPtr control_motor_service_;
    rclcpp::Service<motors::srv::ResetMotors>::SharedPtr reset_motors_service_;
    rclcpp::Service<motors::srv::ReadMotors>::SharedPtr read_motors_service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_leg_publisher_, right_leg_publisher_,
        left_arm_publisher_, right_arm_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_leg_subscription_,
        right_leg_subscription_, left_arm_subscription_, right_arm_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<Decouple> ankle_decouple_;

    void publish_left_leg() {
        Eigen::VectorXd q(2);
        q << left_leg_motors[4]->get_motor_pos(), left_leg_motors[5]->get_motor_pos();
        Eigen::VectorXd vel(2);
        vel << left_leg_motors[4]->get_motor_spd(), left_leg_motors[5]->get_motor_spd();
        Eigen::VectorXd tau(2);
        tau << left_leg_motors[4]->get_motor_current(), left_leg_motors[5]->get_motor_current();
        ankle_decouple_->getForwardQVT(q, vel, tau, true);
        auto left_message = sensor_msgs::msg::JointState();
        left_message.header.stamp = this->now();
        left_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        left_message.position = {left_leg_motors[0]->get_motor_pos() - joint_default_angle_[0],
                                 left_leg_motors[1]->get_motor_pos() - joint_default_angle_[1],
                                 left_leg_motors[2]->get_motor_pos() - joint_default_angle_[2],
                                 left_leg_motors[3]->get_motor_pos() - joint_default_angle_[3],
                                 q[0] - joint_default_angle_[4],
                                 q[1] - joint_default_angle_[5]};
        left_message.velocity = {left_leg_motors[0]->get_motor_spd(),
                                 left_leg_motors[1]->get_motor_spd(),
                                 left_leg_motors[2]->get_motor_spd(),
                                 left_leg_motors[3]->get_motor_spd(),
                                 vel[0],
                                 vel[1]};
        left_message.effort = {left_leg_motors[0]->get_motor_current(),
                               left_leg_motors[1]->get_motor_current(),
                               left_leg_motors[2]->get_motor_current(),
                               left_leg_motors[3]->get_motor_current(),
                               tau[0],
                               tau[1]};
        left_leg_publisher_->publish(left_message);
    }

    void publish_right_leg() {
        Eigen::VectorXd q(2);
        q << right_leg_motors[4]->get_motor_pos(), right_leg_motors[5]->get_motor_pos();
        Eigen::VectorXd vel(2);
        vel << right_leg_motors[4]->get_motor_spd(), right_leg_motors[5]->get_motor_spd();
        Eigen::VectorXd tau(2);
        tau << right_leg_motors[4]->get_motor_current(), right_leg_motors[5]->get_motor_current();
        ankle_decouple_->getForwardQVT(q, vel, tau, false);
        auto right_message = sensor_msgs::msg::JointState();
        right_message.header.stamp = this->now();
        right_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        right_message.position = {right_leg_motors[0]->get_motor_pos() - joint_default_angle_[6],
                                  right_leg_motors[1]->get_motor_pos() - joint_default_angle_[7],
                                  right_leg_motors[2]->get_motor_pos() - joint_default_angle_[8],
                                  right_leg_motors[3]->get_motor_pos() - joint_default_angle_[9],
                                  q[0] - joint_default_angle_[10],
                                  q[1] - joint_default_angle_[11],
                                  right_leg_motors[6]->get_motor_pos() - joint_default_angle_[12]};
        right_message.velocity = {right_leg_motors[0]->get_motor_spd(),
                                  right_leg_motors[1]->get_motor_spd(),
                                  right_leg_motors[2]->get_motor_spd(),
                                  right_leg_motors[3]->get_motor_spd(),
                                  vel[0],
                                  vel[1],
                                  right_leg_motors[6]->get_motor_spd()};
        right_message.effort = {right_leg_motors[0]->get_motor_current(),
                                right_leg_motors[1]->get_motor_current(),
                                right_leg_motors[2]->get_motor_current(),
                                right_leg_motors[3]->get_motor_current(),
                                tau[0],
                                tau[1],
                                right_leg_motors[6]->get_motor_current()};
        right_leg_publisher_->publish(right_message);
    }

    void publish_left_arm() {
        auto left_message = sensor_msgs::msg::JointState();
        left_message.header.stamp = this->now();
        left_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        left_message.position = {left_arm_motors[0]->get_motor_pos() - joint_default_angle_[13],
                                 left_arm_motors[1]->get_motor_pos() - joint_default_angle_[14],
                                 left_arm_motors[2]->get_motor_pos() - joint_default_angle_[15],
                                 left_arm_motors[3]->get_motor_pos() - joint_default_angle_[16],
                                 left_arm_motors[4]->get_motor_pos() - joint_default_angle_[17]};
        left_message.velocity = {left_arm_motors[0]->get_motor_spd(), left_arm_motors[1]->get_motor_spd(),
                                 left_arm_motors[2]->get_motor_spd(), left_arm_motors[3]->get_motor_spd(),
                                 left_arm_motors[4]->get_motor_spd()};
        left_message.effort = {
            left_arm_motors[0]->get_motor_current(), left_arm_motors[1]->get_motor_current(),
            left_arm_motors[2]->get_motor_current(), left_arm_motors[3]->get_motor_current(),
            left_arm_motors[4]->get_motor_current()};
        left_arm_publisher_->publish(left_message);
    }

    void publish_right_arm() {
        auto right_message = sensor_msgs::msg::JointState();
        right_message.header.stamp = this->now();
        right_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        right_message.position = {right_arm_motors[0]->get_motor_pos() - joint_default_angle_[18],
                                  right_arm_motors[1]->get_motor_pos() - joint_default_angle_[19],
                                  right_arm_motors[2]->get_motor_pos() - joint_default_angle_[20],
                                  right_arm_motors[3]->get_motor_pos() - joint_default_angle_[21],
                                  right_arm_motors[4]->get_motor_pos() - joint_default_angle_[22]};
        right_message.velocity = {right_arm_motors[0]->get_motor_spd(), right_arm_motors[1]->get_motor_spd(),
                                  right_arm_motors[2]->get_motor_spd(), right_arm_motors[3]->get_motor_spd(),
                                  right_arm_motors[4]->get_motor_spd()};
        right_message.effort = {
            right_arm_motors[0]->get_motor_current(), right_arm_motors[1]->get_motor_current(),
            right_arm_motors[2]->get_motor_current(), right_arm_motors[3]->get_motor_current(),
            right_arm_motors[4]->get_motor_current()};
        right_arm_publisher_->publish(right_message);
    }

    void subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        Eigen::VectorXd q(2);
        q << msg->position[4] + joint_default_angle_[4], msg->position[5] + joint_default_angle_[5];
        Eigen::VectorXd vel(2);
        vel << msg->velocity[4], msg->velocity[5];
        Eigen::VectorXd tau(2);
        tau << msg->effort[4], msg->effort[5];
        ankle_decouple_->getDecoupleQVT(q, vel, tau, true);
        {
            std::unique_lock lock(left_leg_mutex_);
            for (int i = can0_startID_; i <= can0_endID_; i++) {
                if (i - can0_startID_ == 4 || i - can0_startID_ == 5) {
                    left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                        q[i - can0_startID_ - 4], vel[i - can0_startID_ - 4], kp_[i - can0_startID_],
                        kd_[i - can0_startID_], tau[i - can0_startID_ - 4]);
                } else {
                    left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                        msg->position[i - can0_startID_] + joint_default_angle_[i - can0_startID_],
                        msg->velocity[i - can0_startID_], kp_[i - can0_startID_], kd_[i - can0_startID_],
                        msg->effort[i - can0_startID_]);
                }
                Timer::ThreadSleepForUs(200);
            }
        }
        publish_left_leg();
    }

    void subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        Eigen::VectorXd q(2);
        q << msg->position[4] + joint_default_angle_[10], msg->position[5] + joint_default_angle_[11];
        Eigen::VectorXd vel(2);
        vel << msg->velocity[4], msg->velocity[5];
        Eigen::VectorXd tau(2);
        tau << msg->effort[4], msg->effort[5];
        ankle_decouple_->getDecoupleQVT(q, vel, tau, false);
        {
            std::unique_lock lock(right_leg_mutex_);
            for (int i = can1_startID_; i <= can1_endID_; i++) {
                if (i - can1_startID_ == 4 || i - can1_startID_ == 5) {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        q[i - can1_startID_ - 4], vel[i - can1_startID_ - 4], kp_[i - can1_startID_],
                        kd_[i - can1_startID_], tau[i - can1_startID_ - 4]);
                } else {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        msg->position[i - can1_startID_] + joint_default_angle_[i - can1_startID_ + 6],
                        msg->velocity[i - can1_startID_], kp_[i - can1_startID_], kd_[i - can1_startID_],
                        msg->effort[i - can1_startID_]);
                }
                Timer::ThreadSleepForUs(200);
            }
        }
        publish_right_leg();
    }

    void subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        {
            std::unique_lock lock(left_arm_mutex_);
            for (int i = can2_startID_; i <= can2_endID_; i++) {
                left_arm_motors[i - can2_startID_]->MotorMitModeCmd(
                    msg->position[i - can2_startID_] + joint_default_angle_[i - can2_startID_ + 13],
                    msg->velocity[i - can2_startID_], kp_[i - can2_startID_ + 7], kd_[i - can2_startID_ + 7],
                    msg->effort[i - can2_startID_]);
                Timer::ThreadSleepForUs(200);
            }
        }
        publish_left_arm();
    }

    void subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
        {
            std::unique_lock lock(right_arm_mutex_);
            for (int i = can3_startID_; i <= can3_endID_; i++) {
                right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                    msg->position[i - can3_startID_] + joint_default_angle_[i - can3_startID_ + 18],
                    msg->velocity[i - can3_startID_], kp_[i - can3_startID_ + 7], kd_[i - can3_startID_ + 7],
                    msg->effort[i - can3_startID_]);
                Timer::ThreadSleepForUs(200);
            }
        }
        publish_right_arm();
    }

    void reset_motors(const std::shared_ptr<motors::srv::ResetMotors::Request> request,
                      std::shared_ptr<motors::srv::ResetMotors::Response> response) {
        try {
            {
                std::scoped_lock lock(left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_);
                for (int i = can0_startID_; i <= can0_endID_; i++) {
                    if (i - can0_startID_ == 4 || i - can0_startID_ == 5) {
                        left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                            left_ankle_motors_default_angle_[i - can0_startID_ - 4], 0, 20, 2, 0);
                    } else {
                        left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                            joint_default_angle_[i - can0_startID_], 0, 20, 2, 0);
                    }
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can1_startID_; i <= can1_endID_; i++) {
                    if (i - can1_startID_ == 4 || i - can1_startID_ == 5) {
                        right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                            right_ankle_motors_default_angle_[i - can1_startID_ - 4], 0, 20, 2, 0);
                    } else {
                        right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                            joint_default_angle_[i - can1_startID_ + 6], 0, 20, 2, 0);
                    }
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can2_startID_; i <= can2_endID_; i++) {
                    left_arm_motors[i - can2_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can2_startID_ + 13], 0, 20, 2, 0);
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can3_startID_; i <= can3_endID_; i++) {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can3_startID_ + 18], 0, 20, 2, 0);
                    Timer::ThreadSleepFor(1);
                }
                Timer::ThreadSleepFor(2000);
                for (int i = can0_startID_; i <= can0_endID_; i++) {
                    if (i - can0_startID_ == 4 || i - can0_startID_ == 5) {
                        left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                            left_ankle_motors_default_angle_[i - can0_startID_ - 4], 0,
                            kp_[i - can0_startID_], kd_[i - can0_startID_], 0);
                    } else {
                        left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                            joint_default_angle_[i - can0_startID_], 0, kp_[i - can0_startID_],
                            kd_[i - can0_startID_], 0);
                    }
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can1_startID_; i <= can1_endID_; i++) {
                    if (i - can1_startID_ == 4 || i - can1_startID_ == 5) {
                        right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                            right_ankle_motors_default_angle_[i - can1_startID_ - 4], 0,
                            kp_[i - can1_startID_], kd_[i - can1_startID_], 0);
                    } else {
                        right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                            joint_default_angle_[i - can1_startID_ + 6], 0, kp_[i - can1_startID_],
                            kd_[i - can1_startID_], 0);
                    }
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can2_startID_; i <= can2_endID_; i++) {
                    left_arm_motors[i - can2_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can2_startID_ + 13], 0, kp_[i - can2_startID_ + 7],
                        kd_[i - can2_startID_ + 7], 0);
                    Timer::ThreadSleepFor(1);
                }
                for (int i = can3_startID_; i <= can3_endID_; i++) {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can3_startID_ + 18], 0, kp_[i - can3_startID_ + 7],
                        kd_[i - can3_startID_ + 7], 0);
                    Timer::ThreadSleepFor(1);
                }
            }
            Timer::ThreadSleepFor(100);
            publish_left_leg();
            publish_right_leg();
            publish_left_arm();
            publish_right_arm();

            response->success = true;
            response->message = "Motors reset successfully";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
        }
    }

    void read_motors(const std::shared_ptr<motors::srv::ReadMotors::Request> request,
                     std::shared_ptr<motors::srv::ReadMotors::Response> response) {
        try {
            {
                std::scoped_lock lock(left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_);
                for (int i = can0_startID_; i <= can0_endID_; i++) {
                    left_leg_motors[i - can0_startID_]->refresh_motor_status();
                    Timer::ThreadSleepForUs(400);
                }
                for (int i = can1_startID_; i <= can1_endID_; i++) {
                    right_leg_motors[i - can1_startID_]->refresh_motor_status();
                    Timer::ThreadSleepForUs(400);
                }
                for (int i = can2_startID_; i <= can2_endID_; i++) {
                    left_arm_motors[i - can2_startID_]->refresh_motor_status();
                    Timer::ThreadSleepForUs(400);
                }
                for (int i = can3_startID_; i <= can3_endID_; i++) {
                    right_arm_motors[i - can3_startID_]->refresh_motor_status();
                    Timer::ThreadSleepForUs(400);
                }
            }
            Timer::ThreadSleepFor(100);
            publish_left_leg();
            publish_right_leg();
            publish_left_arm();
            publish_right_arm();

            response->success = true;
            response->message = "Motors read successfully";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
        }
    }

    void control_motor(const std::shared_ptr<motors::srv::ControlMotor::Request> request,
                       std::shared_ptr<motors::srv::ControlMotor::Response> response) {
        int motor_id = request->motor_id;
        int can_id = request->can_id;
        float position = request->position;
        float velocity = request->velocity;
        float effort = request->effort;

        try {
            if (can_id == 0 && motor_id >= can0_startID_ && motor_id <= can0_endID_) {
                std::unique_lock lock(left_leg_mutex_);
                left_leg_motors[motor_id - can0_startID_]->MotorMitModeCmd(
                    position, velocity, kp_[motor_id - can0_startID_], kd_[motor_id - can0_startID_], effort);
                response->success = true;
                response->message = "Send sucessfully";
            } else if (can_id == 1 && motor_id >= can1_startID_ && motor_id <= can1_endID_) {
                std::unique_lock lock(right_leg_mutex_);
                right_leg_motors[motor_id - can1_startID_]->MotorMitModeCmd(
                    position, velocity, kp_[motor_id - can1_startID_], kd_[motor_id - can1_startID_], effort);
                response->success = true;
                response->message = "Send sucessfully";
            } else if (can_id == 2 && motor_id >= can2_startID_ && motor_id <= can2_endID_) {
                std::unique_lock lock(left_arm_mutex_);
                left_arm_motors[motor_id - can2_startID_]->MotorMitModeCmd(
                    position, velocity, kp_[motor_id - can2_startID_ + 7], kd_[motor_id - can2_startID_ + 7],
                    effort);
                response->success = true;
                response->message = "Send sucessfully";
            } else if (can_id == 3 && motor_id >= can3_startID_ && motor_id <= can3_endID_) {
                std::unique_lock lock(right_arm_mutex_);
                right_arm_motors[motor_id - can3_startID_]->MotorMitModeCmd(
                    position, velocity, kp_[motor_id - can3_startID_ + 7], kd_[motor_id - can3_startID_ + 7],
                    effort);
                response->success = true;
                response->message = "Send sucessfully";
            } else {
                response->success = false;
                response->message = "Invalid motor ID";
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}