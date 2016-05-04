#ifndef LCM2ROSCONTROL_HPP
#define LCM2ROSCONTROL_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/force_torque_t.hpp"
#include "lcmtypes/bot_core/six_axis_force_torque_array_t.hpp"
#include "lcmtypes/bot_core/six_axis_force_torque_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/ins_t.hpp"
#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/drc/behavior_transition_t.hpp"

#include <set>
#include <string>
#include <vector>
#include <chrono>

namespace valkyrie_translator
{

  enum class Behavior {
    FREEZE,
    POSITION_CONTROL,
    NORMAL
  };

  struct joint_gains {
    double k_q_p; // corresponds to kp_position in drcsim API
    double k_q_i; // corresponds to ki_position in drcsim API
    double k_qd_p; // corresponds to kp_velocity in drcsim API
    double k_f_p;
    double ff_qd; // maps to kd_position in drcsim API (there isnt an equivalent gain in the bdi api)
    double ff_qd_d;
    double ff_f_d;
    double ff_const;
  };

   struct joint_command {
    double position;
    double velocity;
    double effort;

    joint_gains gains;
   };

   class LCM2ROSControl;

   /* Manages subscription for the LCM2ROSControl class.
      Presently just a hack to resolve pluginlib compatibility issues... */
   class LCM2ROSControl_LCMHandler
   {
   public:
        LCM2ROSControl_LCMHandler(LCM2ROSControl& parent);
        virtual ~LCM2ROSControl_LCMHandler();
        void jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const bot_core::atlas_command_t* msg);
        void behaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                             const drc::behavior_transition_t* msg);
        void update();
   private:
        LCM2ROSControl& parent_;
        // If the subscription is created on LCM2ROSControl's lcm_ object, we see pluginlib
        // compatibility problems. Mysterious and scary...
        std::shared_ptr<lcm::LCM> lcm_;
   };

   class LCM2ROSControl : public controller_interface::Controller<hardware_interface::EffortJointInterface>
   {
   public:
        LCM2ROSControl();
        virtual ~LCM2ROSControl();

        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);

        // Public so it can be modified by the LCMHandler. Should eventually create
        // a friend class arrangement to make this private again.
        std::map<std::string, joint_command> latest_commands;
        bool publishCoreRobotState = true;
        bool publish_est_robot_state = false;
        bool applyCommands = false;
        bool publish_debug_data = false;


        double FORCE_CONTROL_ALLOWABLE_POSITION_ERR_BOUND = 0.1;
        double FORCE_CONTROL_MAX_CHANGE = 100.0;
        double DEFAULT_MIN_POSITION = -M_PI;
        double DEFAULT_MAX_POSITION = M_PI;
        double DEFAULT_MAX_EFFORT = 1000.0;

        std::map<std::string, joint_limits_interface::JointLimits> joint_limits;
        void transitionTo(Behavior new_behavior, ros::Duration transition_duration);

   protected:
        virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                         ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                         std::set<std::string>& claimed_resources) override;

   private:
        boost::shared_ptr<lcm::LCM> lcm_;
        std::shared_ptr<LCM2ROSControl_LCMHandler> handler_;

        std::map<std::string, hardware_interface::JointHandle> effortJointHandles;
        std::map<std::string, hardware_interface::JointHandle> positionJointHandles;
        std::map<std::string, hardware_interface::ImuSensorHandle> imuSensorHandles;
        std::map<std::string, hardware_interface::ForceTorqueSensorHandle> forceTorqueHandles;

        ros::Time last_update;

        Behavior current_behavior = Behavior::FREEZE;
        Behavior previous_behavior = Behavior::FREEZE;
        std::map<std::string, double> latched_positions;
        ros::Time last_transition_start_time;
        ros::Duration last_transition_duration;
        std::map<Behavior, std::map<std::string, joint_gains> > behavior_gain_overrides;

        void latchCurrentPositions();
        bool loadBehaviorGainOverrides(const std::vector<std::string>& effort_names, const ros::NodeHandle& controller_nh);
        double commandPosition(const std::string& joint_name, const joint_command& command, const Behavior& behavior);
        double commandEffort(const std::string& joint_name, const hardware_interface::JointHandle& joint_handle, const joint_command& command, const double dt, const Behavior& behavior);
        double currentTransitionRatio();
   };
}
#endif
