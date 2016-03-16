// Copyright 2016 Wolfgang Merkt

/**
 * Listens for joint position goal commands for the position controlled joints, and feeds them to the robot.
 *
 * Forwards joint state over LCM in appropriate status messages.
 *
 * Runs at 500 Hz in the Valkyrie ros_control main loop as a plugin.
 *
 * Based on LCM2ROSControl originally by gizatt@mit.edu
 *
 * wolfgang.merkt@ed.ac.uk, 201603**
 */

#include <map>
#include <string>
#include <algorithm>
#include <vector>
#include <set>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"

inline double clamp(double x, double lower, double upper) {
    return x < lower ? lower : (x > upper ? upper : x);  // TODO: move to std::min std::max
}

namespace valkyrie_translator {
    class JointPositionGoalController;

    class JointPositionGoalController_LCMHandler {
    public:
        explicit JointPositionGoalController_LCMHandler(JointPositionGoalController &parent);

        virtual ~JointPositionGoalController_LCMHandler();

        void jointPositionGoalHandler(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                      const bot_core::robot_state_t *msg);

        void update();

    private:
        JointPositionGoalController &parent_;
        std::shared_ptr<lcm::LCM> lcm_;
    };

    class JointPositionGoalController
            : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
        friend class JointPositionGoalController_LCMHandler;

    public:
        JointPositionGoalController();

        virtual ~JointPositionGoalController();

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &period);

        void stopping(const ros::Time &time);

    protected:
        bool initRequest(hardware_interface::RobotHW *robot_hw,
                         ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh,
                         std::set<std::string> &claimed_resources) override;

    private:
        boost::shared_ptr<lcm::LCM> lcm_;
        std::shared_ptr<JointPositionGoalController_LCMHandler> handler_;

        std::map<std::string, hardware_interface::JointHandle> positionJointHandles_;

        // Limits: joint position and velocity limits
        std::map<std::string, joint_limits_interface::JointLimits> joint_limits_;
        double max_joint_velocity_;

        std::map<std::string, double> q_measured_;
        std::map<std::string, double> q_delta_;
        std::map<std::string, double> q_start_;

        double q_move_time_;
        std::map<std::string, double> latest_commands_;

        ros::Time last_update_;
    };

    JointPositionGoalController::JointPositionGoalController() { }

    JointPositionGoalController::~JointPositionGoalController() { }

    bool JointPositionGoalController::initRequest(hardware_interface::RobotHW *robot_hw,
                                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh,
                                                  std::set<std::string> &claimed_resources) {
        // check if construction finished cleanly
        if (state_ != CONSTRUCTED) {
            ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
            return false;
        }

        // setup LCM
        lcm_ = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
        if (!lcm_->good()) {
            std::cerr << "ERROR: lcm is not good()" << std::endl;
            return false;
        }
        handler_ = std::shared_ptr<JointPositionGoalController_LCMHandler>(
                new JointPositionGoalController_LCMHandler(*this));

        // Check which joints we have been assigned to
        // If we have joints assigned to just us, claim those, otherwise claim all
        std::vector<std::string> joint_names_;
        if (!controller_nh.getParam("joints", joint_names_))
            ROS_INFO_STREAM("Could not get assigned list of joints, will resume to claim all");

        auto n_joints_ = joint_names_.size();
        bool use_joint_selection = true;
        if (n_joints_ == 0)
            use_joint_selection = false;

        // Retrieve maximum joint velocities in degrees per second and convert to radians per tick
        if (!controller_nh.getParam("joint_velocity_limit", max_joint_velocity_)) {
            ROS_WARN("Cannot retrieve desired maximum joint velocity from param server, defaulting to 10deg/s");
            max_joint_velocity_ = 10;  // Default to 10deg/s if not set
        }
        ROS_INFO_STREAM("Maximum joint velocity: " << max_joint_velocity_ << "deg/s");

        // Retrieve joint limits from parameter server
        for (auto const &joint_name : joint_names_) {
            joint_limits_interface::JointLimits limits;
            if (!getJointLimits(joint_name, controller_nh, limits))
                ROS_ERROR_STREAM("Cannot read joint limits for joint " << joint_name << " from param server");

            joint_limits_.insert(std::make_pair(joint_name, limits));
            ROS_INFO_STREAM("Joint Position Limits: " << joint_name << " has lower limit " << limits.min_position <<
                            " and upper limit " <<
                            limits.max_position);
        }

        // get a pointer to the position interface
        hardware_interface::PositionJointInterface *position_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (!position_hw) {
            ROS_ERROR(
                    "This controller requires a hardware interface of type hardware_interface::PositionJointInterface.");
            return false;
        }

        position_hw->clearClaims();
        const std::vector<std::string> &positionNames = position_hw->getNames();
        // initialize command buffer for each joint we found on the HW
        for (unsigned int i = 0; i < positionNames.size(); i++) {
            if (use_joint_selection &&
                std::find(joint_names_.begin(), joint_names_.end(), positionNames[i]) == joint_names_.end())
                continue;

            positionJointHandles_[positionNames[i]] = position_hw->getHandle(positionNames[i]);
            latest_commands_[positionNames[i]] = 0.0;
        }

        auto position_hw_claims = position_hw->getClaims();
        claimed_resources.insert(position_hw_claims.begin(), position_hw_claims.end());
        position_hw->clearClaims();

        // success
        state_ = INITIALIZED;
        ROS_INFO_STREAM(
                "JointPositionGoalController ON with " << claimed_resources.size() << " claimed resources:" << std::endl
                << position_hw_claims.size() << " position-controlled joints" << std::endl);
        return true;
    }

    void JointPositionGoalController::starting(const ros::Time &time) {
        last_update_ = time;
    }

    void JointPositionGoalController::update(const ros::Time &time, const ros::Duration &period) {
        handler_->update();
        lcm_->handleTimeout(0);

        double dt = (time - last_update_).toSec();
        last_update_ = time;
        int64_t utime = (int64_t) (time.toSec() * 1000000.);

        size_t number_of_joint_interfaces = positionJointHandles_.size();

        // VAL_CORE_ROBOT_STATE
        // push out the joint states for all joints we see advertised
        // and also the commanded torques, for reference
        bot_core::joint_state_t lcm_pose_msg;
        lcm_pose_msg.utime = utime;
        lcm_pose_msg.num_joints = (int16_t) number_of_joint_interfaces;
        lcm_pose_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_pose_msg.joint_position.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_pose_msg.joint_velocity.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_pose_msg.joint_effort.assign(number_of_joint_interfaces, (const float &) 0.);

        // VAL_COMMAND_FEEDBACK
        bot_core::joint_state_t lcm_commanded_msg;
        lcm_commanded_msg.utime = utime;
        lcm_commanded_msg.num_joints = (int16_t) number_of_joint_interfaces;
        lcm_commanded_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_commanded_msg.joint_position.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_commanded_msg.joint_velocity.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_commanded_msg.joint_effort.assign(number_of_joint_interfaces, (const float &) 0.);

        // EST_ROBOT_STATE
        // need to decide what message we're really using for state. for now,
        // assembling this to make director happy
        bot_core::robot_state_t lcm_state_msg;
        lcm_state_msg.utime = utime;
        lcm_state_msg.num_joints = (int16_t) number_of_joint_interfaces;
        lcm_state_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_state_msg.joint_position.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_state_msg.joint_velocity.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_state_msg.joint_effort.assign(number_of_joint_interfaces, (const float &) 0.);
        lcm_state_msg.pose.translation.x = 0.0;
        lcm_state_msg.pose.translation.y = 0.0;
        lcm_state_msg.pose.translation.z = 0.0;
        lcm_state_msg.pose.rotation.w = 1.0;
        lcm_state_msg.pose.rotation.x = 0.0;
        lcm_state_msg.pose.rotation.y = 0.0;
        lcm_state_msg.pose.rotation.z = 0.0;
        lcm_state_msg.twist.linear_velocity.x = 0.0;
        lcm_state_msg.twist.linear_velocity.y = 0.0;
        lcm_state_msg.twist.linear_velocity.z = 0.0;
        lcm_state_msg.twist.angular_velocity.x = 0.0;
        lcm_state_msg.twist.angular_velocity.y = 0.0;
        lcm_state_msg.twist.angular_velocity.z = 0.0;

        // Iterate over all position-controlled joints
        size_t positionJointIndex = 0;
        for (auto iter = positionJointHandles_.begin(); iter != positionJointHandles_.end(); iter++) {
            std::string joint_name = iter->first;
            double &q = q_measured_[joint_name];
            q = iter->second.getPosition();
            double qd = iter->second.getVelocity();

            double &q_desired = latest_commands_[iter->first];
            double &q_delta = q_delta_[joint_name];
            double &q_start = q_start_[joint_name];

            double eta = std::max(0.0, std::min(1.0, dt / q_move_time_));
            double q_command = eta * q_desired + (1 - eta) * q_start;

            // Check that new position is within joint position limits, enforce by clamping
            joint_limits_interface::JointLimits &limits = joint_limits_[joint_name];
            double q_command_with_position_limits_enforced = clamp(q_command, limits.min_position,
                                                                   limits.max_position);

//            std::cout << joint_name << std::endl;
//            std::cout << "q: " << q << std::endl;
//            std::cout << "q_desired: " << q_desired << std::endl;
//            std::cout << "q_start: " << q_start << std::endl;
//            std::cout << "q_command: " << q_command << std::endl;
//            std::cout << "q_command_with_position_limits_enforced: " << q_command_with_position_limits_enforced <<
//            std::endl;
//            std::cout << std::endl;

            // Write command to joint
            iter->second.setCommand(q_command_with_position_limits_enforced);

            lcm_pose_msg.joint_name[positionJointIndex] = joint_name;
            lcm_pose_msg.joint_position[positionJointIndex] = static_cast<float>(q);
            lcm_pose_msg.joint_velocity[positionJointIndex] = static_cast<float>(qd);

            lcm_state_msg.joint_name[positionJointIndex] = joint_name;
            lcm_state_msg.joint_position[positionJointIndex] = static_cast<float>(q);
            lcm_state_msg.joint_velocity[positionJointIndex] = static_cast<float>(qd);

            // republish to guarantee sync
            lcm_commanded_msg.joint_name[positionJointIndex] = joint_name;
            lcm_commanded_msg.joint_position[positionJointIndex] = static_cast<float>(q_command_with_position_limits_enforced);

            positionJointIndex++;
        }

        lcm_->publish("VAL_CORE_ROBOT_STATE", &lcm_pose_msg);
        lcm_->publish("VAL_COMMAND_FEEDBACK", &lcm_commanded_msg);
        lcm_->publish("EST_ROBOT_STATE", &lcm_state_msg);
    }

    void JointPositionGoalController::stopping(const ros::Time &time) { }

    JointPositionGoalController_LCMHandler::JointPositionGoalController_LCMHandler(JointPositionGoalController &parent)
            : parent_(parent) {
        lcm_ = std::shared_ptr<lcm::LCM>(new lcm::LCM);
        if (!lcm_->good()) {
            std::cerr << "ERROR: handler lcm is not good()" << std::endl;
        }
        lcm_->subscribe("JOINT_POSITION_GOAL", &JointPositionGoalController_LCMHandler::jointPositionGoalHandler, this);
    }

    JointPositionGoalController_LCMHandler::~JointPositionGoalController_LCMHandler() { }

    void JointPositionGoalController_LCMHandler::jointPositionGoalHandler(const lcm::ReceiveBuffer *rbuf,
                                                                          const std::string &channel,
                                                                          const bot_core::robot_state_t *msg) {
        // Reset q_move_time_
        parent_.q_move_time_ = 0;

        // Iterate over all received joints
        for (unsigned int i = 0; i < msg->num_joints; ++i) {
            auto search = parent_.latest_commands_.find(msg->joint_name[i]);
            if (search != parent_.latest_commands_.end()) {
                std::string joint_name = msg->joint_name[i];
                ROS_INFO_STREAM(joint_name << " got new q_desired " << msg->joint_position[i]);
                double &q_desired = parent_.latest_commands_[joint_name];
                q_desired = msg->joint_position[i];

                double &q = parent_.q_measured_[joint_name];
                double &q_delta = parent_.q_delta_[joint_name];
                q_delta = q_desired - q;
                double &q_start = parent_.q_start_[joint_name];
                q_start = q;

                // Calculate move time for joint, set controller q_move_time_ to largest value
                double tmp_q_move_time = std::abs(q_delta) / parent_.max_joint_velocity_;
                if (tmp_q_move_time > parent_.q_move_time_)
                    parent_.q_move_time_ = tmp_q_move_time;
            }
        }
    }

    void JointPositionGoalController_LCMHandler::update() {
        lcm_->handleTimeout(0);
    }
}  // namespace valkyrie_translator

PLUGINLIB_EXPORT_CLASS(valkyrie_translator::JointPositionGoalController, controller_interface::ControllerBase)
