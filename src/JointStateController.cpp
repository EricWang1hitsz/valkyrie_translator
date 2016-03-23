// Copyright 2016 Wolfgang Merkt

#include <cstdint>
#include <map>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
#include <memory>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/robot_state_t.hpp"

// TODO: sensor readings e.g. F/T


namespace valkyrie_translator {
    class JointStateController;

    class JointStateController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
    public:
        JointStateController() { }

        bool init(hardware_interface::JointStateInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &period);

        void stopping(const ros::Time &time);

    private:
        std::vector<std::string> joint_names_;
        std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
        std::shared_ptr<lcm::LCM> lcm_;
        unsigned int number_of_joint_interfaces_;

        bot_core::robot_state_t est_robot_state_;
    };


    bool JointStateController::init(hardware_interface::JointStateInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        // Retrieve all joint names from the hardware interface
        joint_names_ = hw->getNames();
        number_of_joint_interfaces_ = static_cast<unsigned int>(joint_names_.size());

        // Setup LCM
        lcm_ = std::shared_ptr<lcm::LCM>(new lcm::LCM);
        if (!lcm_->good()) {
            std::cerr << "ERROR: lcm is not good()" << std::endl;
            return false;
        }

        // Initialise message
        est_robot_state_.utime = 0;
        est_robot_state_.num_joints = (int16_t) number_of_joint_interfaces_;
        est_robot_state_.joint_name.assign(number_of_joint_interfaces_, "");
        est_robot_state_.joint_position.assign(number_of_joint_interfaces_, (const float &) 0.);
        est_robot_state_.joint_velocity.assign(number_of_joint_interfaces_, (const float &) 0.);
        est_robot_state_.joint_effort.assign(number_of_joint_interfaces_, (const float &) 0.);
        est_robot_state_.pose.translation.x = 0.0;
        est_robot_state_.pose.translation.y = 0.0;
        est_robot_state_.pose.translation.z = 0.0;
        est_robot_state_.pose.rotation.w = 1.0;
        est_robot_state_.pose.rotation.x = 0.0;
        est_robot_state_.pose.rotation.y = 0.0;
        est_robot_state_.pose.rotation.z = 0.0;
        est_robot_state_.twist.linear_velocity.x = 0.0;
        est_robot_state_.twist.linear_velocity.y = 0.0;
        est_robot_state_.twist.linear_velocity.z = 0.0;
        est_robot_state_.twist.angular_velocity.x = 0.0;
        est_robot_state_.twist.angular_velocity.y = 0.0;
        est_robot_state_.twist.angular_velocity.z = 0.0;

        // Retrieve joint handles and init est_robot_state_msg_
        for (unsigned int i = 0; i < number_of_joint_interfaces_; i++) {
            ROS_INFO_STREAM("Joint " << joint_names_[i]);  // TODO: debug, remove

            joint_state_handles_.push_back(hw->getHandle(joint_names_[i]));
            est_robot_state_.joint_name[i] = joint_names_[i];
        }

        return true;
    }

    void JointStateController::starting(const ros::Time &time) { }

    void JointStateController::update(const ros::Time &time, const ros::Duration &period) {
        est_robot_state_.utime = static_cast<int64_t>(time.toSec() * 1e6);

        for (unsigned int i = 0; i < number_of_joint_interfaces_; i++) {
            est_robot_state_.joint_position[i] = static_cast<float>(joint_state_handles_[i].getPosition());
            est_robot_state_.joint_velocity[i] = static_cast<float>(joint_state_handles_[i].getVelocity());
            est_robot_state_.joint_effort[i] = static_cast<float>(joint_state_handles_[i].getEffort());
        }

        lcm_->publish("EST_ROBOT_STATE", &est_robot_state_);
    }

    void JointStateController::stopping(const ros::Time &time) { }

}  // namespace valkyrie_translator

PLUGINLIB_EXPORT_CLASS(valkyrie_translator::JointStateController, controller_interface::ControllerBase)
