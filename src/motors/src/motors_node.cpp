#include "motors_node.hpp"

void MotorsNode::publish_left_leg() {
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

void MotorsNode::publish_right_leg() {
    Eigen::VectorXd q(2);
    q << -right_leg_motors[4]->get_motor_pos(), -right_leg_motors[5]->get_motor_pos();
    Eigen::VectorXd vel(2);
    vel << -right_leg_motors[4]->get_motor_spd(), -right_leg_motors[5]->get_motor_spd();
    Eigen::VectorXd tau(2);
    tau << -right_leg_motors[4]->get_motor_current(), -right_leg_motors[5]->get_motor_current();
    ankle_decouple_->getForwardQVT(q, vel, tau, false);
    auto right_message = sensor_msgs::msg::JointState();
    right_message.header.stamp = this->now();
    right_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    right_message.position = {right_leg_motors[0]->get_motor_pos() - joint_default_angle_[6],
                              right_leg_motors[1]->get_motor_pos() - joint_default_angle_[7],
                              -(right_leg_motors[2]->get_motor_pos() + joint_default_angle_[8]),
                              -(right_leg_motors[3]->get_motor_pos() + joint_default_angle_[9]),
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

void MotorsNode::publish_left_arm() {
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
    left_message.effort = {left_arm_motors[0]->get_motor_current(), left_arm_motors[1]->get_motor_current(),
                           left_arm_motors[2]->get_motor_current(), left_arm_motors[3]->get_motor_current(),
                           left_arm_motors[4]->get_motor_current()};
    left_arm_publisher_->publish(left_message);
}

void MotorsNode::publish_right_arm() {
    auto right_message = sensor_msgs::msg::JointState();
    right_message.header.stamp = this->now();
    right_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    right_message.position = {-(right_arm_motors[0]->get_motor_pos() + joint_default_angle_[18]),
                              right_arm_motors[1]->get_motor_pos() - joint_default_angle_[19],
                              right_arm_motors[2]->get_motor_pos() - joint_default_angle_[20],
                              -(right_arm_motors[3]->get_motor_pos() + joint_default_angle_[21]),
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

void MotorsNode::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
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

void MotorsNode::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
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
                    -q[i - can1_startID_ - 4], -vel[i - can1_startID_ - 4], kp_[i - can1_startID_],
                    kd_[i - can1_startID_], -tau[i - can1_startID_ - 4]);
            } else if (i - can1_startID_ == 2 || i - can1_startID_ == 3) {
                right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                    -(msg->position[i - can1_startID_] + joint_default_angle_[i - can1_startID_ + 6]),
                    -msg->velocity[i - can1_startID_], kp_[i - can1_startID_], kd_[i - can1_startID_],
                    -msg->effort[i - can1_startID_]);
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

void MotorsNode::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
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

void MotorsNode::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    {
        std::unique_lock lock(right_arm_mutex_);
        for (int i = can3_startID_; i <= can3_endID_; i++) {
            if (i - can3_startID_ == 0 || i - can3_startID_ == 3) {
                right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                    -(msg->position[i - can3_startID_] + joint_default_angle_[i - can3_startID_ + 18]),
                    -msg->velocity[i - can3_startID_], kp_[i - can3_startID_ + 7], kd_[i - can3_startID_ + 7],
                    -msg->effort[i - can3_startID_]);
            } else {
                right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                    msg->position[i - can3_startID_] + joint_default_angle_[i - can3_startID_ + 18],
                    msg->velocity[i - can3_startID_], kp_[i - can3_startID_ + 7], kd_[i - can3_startID_ + 7],
                    msg->effort[i - can3_startID_]);
            }
            Timer::ThreadSleepForUs(200);
        }
    }
    publish_right_arm();
}

void MotorsNode::reset_motors(const std::shared_ptr<motors::srv::ResetMotors::Request> request,
                              std::shared_ptr<motors::srv::ResetMotors::Response> response) {
    try {
        {
            std::scoped_lock lock(left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_);
            for (int i = can0_startID_; i <= can0_endID_; i++) {
                if (i - can0_startID_ == 4 || i - can0_startID_ == 5) {
                    left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                        left_ankle_motors_default_angle_[i - can0_startID_ - 4], 0, 40, 2, 0);
                } else {
                    left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can0_startID_], 0, 40, 2, 0);
                }
                Timer::ThreadSleepFor(1);
            }
            for (int i = can1_startID_; i <= can1_endID_; i++) {
                if (i - can1_startID_ == 4 || i - can1_startID_ == 5) {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        -right_ankle_motors_default_angle_[i - can1_startID_ - 4], 0, 40, 2, 0);
                } else if (i - can1_startID_ == 2 || i - can1_startID_ == 3) {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        -joint_default_angle_[i - can1_startID_ + 6], 0, 40, 2, 0);
                } else {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can1_startID_ + 6], 0, 40, 2, 0);
                }
                Timer::ThreadSleepFor(1);
            }
            for (int i = can2_startID_; i <= can2_endID_; i++) {
                left_arm_motors[i - can2_startID_]->MotorMitModeCmd(
                    joint_default_angle_[i - can2_startID_ + 13], 0, 40, 2, 0);
                Timer::ThreadSleepFor(1);
            }
            for (int i = can3_startID_; i <= can3_endID_; i++) {
                if (i - can3_startID_ == 0 || i - can3_startID_ == 3) {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        -joint_default_angle_[i - can3_startID_ + 18], 0, 40, 2, 0);
                } else {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can3_startID_ + 18], 0, 40, 2, 0);
                }
                Timer::ThreadSleepFor(1);
            }
            Timer::ThreadSleepFor(2000);
            for (int i = can0_startID_; i <= can0_endID_; i++) {
                if (i - can0_startID_ == 4 || i - can0_startID_ == 5) {
                    left_leg_motors[i - can0_startID_]->MotorMitModeCmd(
                        left_ankle_motors_default_angle_[i - can0_startID_ - 4], 0, kp_[i - can0_startID_],
                        kd_[i - can0_startID_], 0);
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
                        -right_ankle_motors_default_angle_[i - can1_startID_ - 4], 0, kp_[i - can1_startID_],
                        kd_[i - can1_startID_], 0);
                } else if (i - can1_startID_ == 2 || i - can1_startID_ == 3) {
                    right_leg_motors[i - can1_startID_]->MotorMitModeCmd(
                        -joint_default_angle_[i - can1_startID_ + 6], 0, kp_[i - can1_startID_],
                        kd_[i - can1_startID_], 0);
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
                if (i - can3_startID_ == 0 || i - can3_startID_ == 3) {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        -joint_default_angle_[i - can3_startID_ + 18], 0, kp_[i - can3_startID_ + 7],
                        kd_[i - can3_startID_ + 7], 0);
                } else {
                    right_arm_motors[i - can3_startID_]->MotorMitModeCmd(
                        joint_default_angle_[i - can3_startID_ + 18], 0, kp_[i - can3_startID_ + 7],
                        kd_[i - can3_startID_ + 7], 0);
                }
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

void MotorsNode::read_motors(const std::shared_ptr<motors::srv::ReadMotors::Request> request,
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

void MotorsNode::control_motor(const std::shared_ptr<motors::srv::ControlMotor::Request> request,
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

void MotorsNode::set_zeros(const std::shared_ptr<motors::srv::SetZeros::Request> request,
                           std::shared_ptr<motors::srv::SetZeros::Response> response) {
    try {
        {
            std::scoped_lock lock(left_leg_mutex_, right_leg_mutex_, left_arm_mutex_, right_arm_mutex_);
            for (int i = can0_startID_; i <= can0_endID_; i++) {
                left_leg_motors[i - can0_startID_]->MotorSetZero();
                Timer::ThreadSleepFor(1);
            }
            for (int i = can1_startID_; i <= can1_endID_; i++) {
                right_leg_motors[i - can1_startID_]->MotorSetZero();
                Timer::ThreadSleepFor(1);
            }
            for (int i = can2_startID_; i <= can2_endID_; i++) {
                left_arm_motors[i - can2_startID_]->MotorSetZero();
                Timer::ThreadSleepFor(1);
            }
            for (int i = can3_startID_; i <= can3_endID_; i++) {
                right_arm_motors[i - can3_startID_]->MotorSetZero();
                Timer::ThreadSleepFor(1);
            }
        }
        Timer::ThreadSleepFor(100);
        response->success = true;
        response->message = "Zeros set successfully";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}