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

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/force_torque_t.hpp"
#include "lcmtypes/bot_core/six_axis_force_torque_array_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/ins_t.hpp"
#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/bot_core/atlas_command_t.hpp"

#include <set>
#include <string>
#include <vector>
#include <map>

inline double clamp(double x, double lower, double upper) {
    return x < lower ? lower : (x > upper ? upper : x);
}

inline double toRad(double deg) {
    return (deg * M_PI / 180);
}


namespace valkyrie_translator {
    class JointPositionGoalController;

    /* Manages subscription for the JointPositionGoalController class.
       Presently just a hack to resolve pluginlib compatibility issues... */
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
    public:
        JointPositionGoalController();

        virtual ~JointPositionGoalController();

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &period);

        void stopping(const ros::Time &time);

        // Public so it can be modified by the LCMHandler. Should eventually create
        // a friend class arrangement to make this private again.
        std::map<std::string, double> latest_commands;

    protected:
        bool initRequest(hardware_interface::RobotHW *robot_hw,
                         ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh,
                         std::set<std::string> &claimed_resources) override;

    private:
        boost::shared_ptr<lcm::LCM> lcm_;
        std::shared_ptr<JointPositionGoalController_LCMHandler> handler_;

        std::map<std::string, hardware_interface::JointHandle> positionJointHandles;

        std::map<std::string, joint_limits_interface::JointLimits> joint_limits;
        double max_joint_velocity;
        double max_joint_velocity_rad_per_cycle;
        double control_frequency = 500;  // Hz

        ros::Time last_update;
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

        // setup LCM (todo: move to constructor? how to propagate an error then?)
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
        if (!controller_nh.getParam("joint_velocity_limit", max_joint_velocity)) {
            ROS_WARN("Cannot retrieve desired maximum joint velocity from param server, defaulting to 10deg/s");
            max_joint_velocity = 10;  // Default to 10deg/s if not set
        }
        max_joint_velocity_rad_per_cycle = toRad(max_joint_velocity) / control_frequency;
        ROS_INFO_STREAM(
                "Maximum joint velocity: " << max_joint_velocity << "deg/s, " << max_joint_velocity_rad_per_cycle <<
                "rad/cycle");

        // Retrieve joint limits from parameter server
        for (auto const &joint_name: joint_names_) {
            joint_limits_interface::JointLimits limits;
            if (!getJointLimits(joint_name, controller_nh, limits))
                ROS_ERROR_STREAM("Cannot read joint limits for joint " << joint_name << " from param server");

            joint_limits.insert(std::make_pair(joint_name, limits));
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

            positionJointHandles[positionNames[i]] = position_hw->getHandle(positionNames[i]);
            latest_commands[positionNames[i]] = 0.0;
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
        last_update = time;
    }

    void JointPositionGoalController::update(const ros::Time &time, const ros::Duration &period) {
        handler_->update();
        lcm_->handleTimeout(0);

        double dt = (time - last_update).toSec();
        last_update = time;
        int64_t utime = (int64_t) (time.toSec() * 1000000.);

        size_t number_of_joint_interfaces = positionJointHandles.size();

        // VAL_CORE_ROBOT_STATE
        // push out the joint states for all joints we see advertised
        // and also the commanded torques, for reference
        bot_core::joint_state_t lcm_pose_msg;
        lcm_pose_msg.utime = utime;
        lcm_pose_msg.num_joints = number_of_joint_interfaces;
        lcm_pose_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_pose_msg.joint_position.assign(number_of_joint_interfaces, 0.);
        lcm_pose_msg.joint_velocity.assign(number_of_joint_interfaces, 0.);
        lcm_pose_msg.joint_effort.assign(number_of_joint_interfaces, 0.);

        // VAL_COMMAND_FEEDBACK
        bot_core::joint_state_t lcm_commanded_msg;
        lcm_commanded_msg.utime = utime;
        lcm_commanded_msg.num_joints = number_of_joint_interfaces;
        lcm_commanded_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_commanded_msg.joint_position.assign(number_of_joint_interfaces, 0.);
        lcm_commanded_msg.joint_velocity.assign(number_of_joint_interfaces, 0.);
        lcm_commanded_msg.joint_effort.assign(number_of_joint_interfaces, 0.);

        // EST_ROBOT_STATE
        // need to decide what message we're really using for state. for now,
        // assembling this to make director happy
        bot_core::robot_state_t lcm_state_msg;
        lcm_state_msg.utime = utime;
        lcm_state_msg.num_joints = number_of_joint_interfaces;
        lcm_state_msg.joint_name.assign(number_of_joint_interfaces, "");
        lcm_state_msg.joint_position.assign(number_of_joint_interfaces, 0.);
        lcm_state_msg.joint_velocity.assign(number_of_joint_interfaces, 0.);
        lcm_state_msg.joint_effort.assign(number_of_joint_interfaces, 0.);
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


        // TODO: calculate diff and have max velocity as parameter
        // Iterate over all position-controlled joints
        size_t positionJointIndex = 0;
        for (auto iter = positionJointHandles.begin(); iter != positionJointHandles.end(); iter++) {
            std::string joint_name = iter->first;
            double q = iter->second.getPosition();
            double qd = iter->second.getVelocity();  // TODO: check current velocity

            double &q_des = latest_commands[iter->first];

            double vector_to_go = q_des - q;
            // Clamp the overall vector to go with the maximum joint velocity in radian per cycle, i.e. use maximum joint velocity where appropriate
//            double new_position_to_be_commanded =
//                    q + clamp(vector_to_go, -max_joint_velocity_rad_per_cycle, max_joint_velocity_rad_per_cycle); // TODO BUG

            double new_position_to_be_commanded = q + vector_to_go; // TODO: unsafe, but debug

            // Check that new position is within joint position limits, enforce by clamping
            auto limits = joint_limits[joint_name];
            double safe_new_position_to_be_commanded = clamp(new_position_to_be_commanded, limits.min_position,
                                                             limits.max_position);

            // Write command to joint
            iter->second.setCommand(safe_new_position_to_be_commanded);

            lcm_pose_msg.joint_name[positionJointIndex] = joint_name;
            lcm_pose_msg.joint_position[positionJointIndex] = q;
            lcm_pose_msg.joint_velocity[positionJointIndex] = qd;

            lcm_state_msg.joint_name[positionJointIndex] = joint_name;
            lcm_state_msg.joint_position[positionJointIndex] = q;
            lcm_state_msg.joint_velocity[positionJointIndex] = qd;

            // republish to guarantee sync
            lcm_commanded_msg.joint_name[positionJointIndex] = joint_name;
            lcm_commanded_msg.joint_position[positionJointIndex] = safe_new_position_to_be_commanded;

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
        for (unsigned int i = 0; i < msg->num_joints; ++i) {
            auto search = parent_.latest_commands.find(msg->joint_name[i]);
            if (search != parent_.latest_commands.end()) {
                ROS_INFO_STREAM(msg->joint_name[i] << " got new q_des " << msg->joint_position[i]);
                double &command = parent_.latest_commands[msg->joint_name[i]];
                command = msg->joint_position[i];
            }
        }
    }

    void JointPositionGoalController_LCMHandler::update() {
        lcm_->handleTimeout(0);
    }
}  // namespace valkyrie_translator

PLUGINLIB_EXPORT_CLASS(valkyrie_translator::JointPositionGoalController, controller_interface::ControllerBase)
