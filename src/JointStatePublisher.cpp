// Copyright 2016 Wolfgang Merkt

#include <cstdint>
#include <map>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
#include <memory>
#include <chrono>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/six_axis_force_torque_array_t.hpp"
#include "lcmtypes/bot_core/ins_t.hpp"
#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/drc/utime_t.hpp"
#include "lcmtypes/drc/string_t.hpp"



namespace valkyrie_translator {
    class JointStatePublisher;

    class JointStatePublisher : public controller_interface::Controller<hardware_interface::JointStateInterface> {
    public:
        JointStatePublisher() { }

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &period);

        void stopping(const ros::Time &time);

        void updateForceTorqueTareValues(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                         const drc::string_t* msg);

        void publishForceTorqueTareValues(int64_t utime);

    protected:
        bool initRequest(hardware_interface::RobotHW *robot_hw,
                         ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh,
                         std::set<std::string> &claimed_resources) override;

    private:
        void initializeLCMSubscriptions();

        void publishEstRobotState(int64_t utime);

        void publishIMUReadings(int64_t utime);

        void publishForceTorqueReadings(int64_t utime);

        void publishCoreRobotState(int64_t utime);

        // void setupTaredForceTorquePublishing();

        std::vector<std::string> joint_names_;
        std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
        std::map<std::string, hardware_interface::ImuSensorHandle> imu_sensor_handles_;
        std::map<std::string, hardware_interface::ForceTorqueSensorHandle> force_torque_handles_;
        std::map<std::string, std::vector<double>> forceTorqueTaredValueMap;
        std::shared_ptr<lcm::LCM> lcm_;
        unsigned int number_of_joint_interfaces_;

        bot_core::joint_state_t core_robot_state_;
        bot_core::robot_state_t est_robot_state_;

        std::string core_robot_state_channel_;

        double lastTareValuePublishTime = 0;

        bool publish_imu_readings_;
        bool publish_separate_force_torque_readings_;
        bool publish_est_robot_state_;
        bool publishTareValue;
    };


    bool JointStatePublisher::initRequest(hardware_interface::RobotHW *robot_hw,
                                          ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh,
                                          std::set<std::string> &claimed_resources) {
        // Get joint state interface from robot hardware
        hardware_interface::JointStateInterface *hw = robot_hw->get<hardware_interface::JointStateInterface>();
        if (!hw) {
            ROS_ERROR("This controller requires access to the JointStateInterface.");
            return false;
        }

        // Retrieve all joint names from the hardware interface
        std::vector<std::string> available_joint_state_handles = hw->getNames();

        // Filter joint names to exclude the hand motors
        // std::cout << "Number of joints before filtering: " << std::to_string(available_joint_state_handles.size()) << std::endl;
        for (unsigned int i = 0; i < available_joint_state_handles.size(); ++i) {
            if (available_joint_state_handles[i].find("Motor") == std::string::npos)
                joint_names_.push_back(available_joint_state_handles[i]);
        }
        // std::cout << "Number of joints after filtering: " << std::to_string(joint_names_.size()) << std::endl;

        number_of_joint_interfaces_ = static_cast<unsigned int>(joint_names_.size());

        // Setup LCM
        lcm_ = std::shared_ptr<lcm::LCM>(new lcm::LCM);
        if (!lcm_->good()) {
            std::cerr << "ERROR: lcm is not good()" << std::endl;
            return false;
        }

        // setup the LCM subscriptions. Right now only for TARE_FOOT_SENSORS
        this->initializeLCMSubscriptions();

        // Retrieve channel name for the core robot state message (defaults to CORE_ROBOT_STATE)
        if (!controller_nh.getParam("core_robot_state_channel", core_robot_state_channel_))
            core_robot_state_channel_ = "CORE_ROBOT_STATE";
        ROS_INFO_STREAM("Publishing core robot state to " << core_robot_state_channel_);

        // Initialise core robot state message
        core_robot_state_.utime = 0;
        core_robot_state_.num_joints = static_cast<int16_t>(number_of_joint_interfaces_);
        core_robot_state_.joint_name.assign(number_of_joint_interfaces_, "");
        core_robot_state_.joint_position.assign(number_of_joint_interfaces_, (const float &) 0.);
        core_robot_state_.joint_velocity.assign(number_of_joint_interfaces_, (const float &) 0.);
        core_robot_state_.joint_effort.assign(number_of_joint_interfaces_, (const float &) 0.);

        // Initialise robot state message
        est_robot_state_.utime = 0;
        est_robot_state_.num_joints = static_cast<int16_t>(number_of_joint_interfaces_);
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

        // Initialise F/T sensors in est_robot_state_
        est_robot_state_.force_torque.l_foot_force_z = 0.0;
        est_robot_state_.force_torque.l_foot_torque_x = 0.0;
        est_robot_state_.force_torque.l_foot_torque_y = 0.0;
        est_robot_state_.force_torque.r_foot_force_z = 0.0;
        est_robot_state_.force_torque.r_foot_torque_x = 0.0;
        est_robot_state_.force_torque.r_foot_torque_y = 0.0;

        for (unsigned int i = 0; i < 3; i++) {
            est_robot_state_.force_torque.l_hand_force[i] = 0.0;
            est_robot_state_.force_torque.l_hand_torque[i] = 0.0;
            est_robot_state_.force_torque.r_hand_force[i] = 0.0;
            est_robot_state_.force_torque.r_hand_torque[i] = 0.0;
        }

        // Retrieve joint handles and init est_robot_state_msg_
        for (unsigned int i = 0; i < joint_names_.size(); i++) {
            try {
                // ROS_INFO_STREAM("Joint " << joint_names_[i]);
                joint_state_handles_.push_back(hw->getHandle(joint_names_[i]));
                est_robot_state_.joint_name[i] = joint_names_[i];
                core_robot_state_.joint_name[i] = joint_names_[i];
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM("Could not retrieve handle for " << joint_names_[i] << ": " << e.what());
            }
        }

        // Retrieve the force torque sensor interface
        publishTareValue = true;
        hardware_interface::ForceTorqueSensorInterface *force_torque_hw = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();
        if (!force_torque_hw) {
            ROS_ERROR(
                    "This controller requires a hardware interface of type hardware_interface::EffortJointInterface.");
            return false;
        }

        const std::vector<std::string> &force_torque_names = force_torque_hw->getNames();
        for (unsigned int i = 0; i < force_torque_names.size(); i++) {
            try {
                force_torque_handles_.insert(
                        std::make_pair(force_torque_names[i], force_torque_hw->getHandle(force_torque_names[i])));

                // initialize the storing of tared values. Initially all zeros
                forceTorqueTaredValueMap.insert(std::make_pair(force_torque_names[i], std::vector<double>(6)));
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM("Could not retrieve handle for " << force_torque_names[i] << ": " << e.what());
            }
        }

        // Retrieve parameter whether to publish EST_ROBOT_STATE (robot_state_t)
        if (!controller_nh.getParam("publish_est_robot_state", publish_est_robot_state_))
            publish_est_robot_state_ = true;
        if(force_torque_names.size()==0)
        {
            publishTareValue = false;
            publish_est_robot_state_ = false;
        }
        ROS_INFO_STREAM("Publishing EST_ROBOT_STATE: " << std::to_string(publish_imu_readings_));

        // Retrieve parameter whether to publish IMU sensor readings
        if (!controller_nh.getParam("publish_imu_readings", publish_imu_readings_))
            publish_imu_readings_ = true;
        ROS_INFO_STREAM("Publishing IMU readings: " << std::to_string(publish_imu_readings_));

        if (publish_imu_readings_) {
            // Retrieve the IMU sensor interface
            hardware_interface::ImuSensorInterface *imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
            if (!imu_hw) {
                ROS_ERROR(
                        "This controller requires a hardware interface of type hardware_interface::ImuSensorInterface.");
                return false;
            }

            const std::vector<std::string> &imu_names = imu_hw->getNames();
            if(imu_names.size()==0) publish_imu_readings_=false;

            for (unsigned int i = 0; i < imu_names.size(); i++) {
                try {
                    imu_sensor_handles_.insert(std::make_pair(imu_names[i], imu_hw->getHandle(imu_names[i])));
                } catch (const hardware_interface::HardwareInterfaceException& e) {
                    ROS_ERROR_STREAM("Could not retrieve handle for " << imu_names[i] << ": " << e.what());
                }
            }
        }

        // Retrieve parameter whether to publish separate force-torque sensor readings in addition to EST_ROBOT_STATE
        if (!controller_nh.getParam("publish_separate_force_torque_readings", publish_separate_force_torque_readings_))
            publish_separate_force_torque_readings_ = false;
        if(force_torque_names.size()==0) publish_separate_force_torque_readings_ = false;
        ROS_INFO_STREAM("Publishing separate FORCE_TORQUE readings: " <<
                        std::to_string(publish_separate_force_torque_readings_));



        state_ = INITIALIZED;
        return true;
    }

    void JointStatePublisher::starting(const ros::Time &time) { }

    void JointStatePublisher::update(const ros::Time &time, const ros::Duration &period) {
        lcm_->handleTimeout(0);
        int64_t utime = static_cast<int64_t>(time.toSec() * 1e6);

        publishCoreRobotState(utime);

        if (publish_est_robot_state_)
            publishEstRobotState(utime);

        if (publish_imu_readings_)
            publishIMUReadings(utime);

        if (publish_separate_force_torque_readings_)
            publishForceTorqueReadings(utime);

        if((time.toSec() - lastTareValuePublishTime) > 1){
            if(publishTareValue) publishForceTorqueTareValues(utime);
            lastTareValuePublishTime = time.toSec();
        }

    }

    void JointStatePublisher::publishEstRobotState(int64_t utime) {
        est_robot_state_.utime = utime;

        for (unsigned int i = 0; i < number_of_joint_interfaces_; i++) {
            est_robot_state_.joint_position[i] = static_cast<float>(joint_state_handles_[i].getPosition());
            est_robot_state_.joint_velocity[i] = static_cast<float>(joint_state_handles_[i].getVelocity());
            est_robot_state_.joint_effort[i] = static_cast<float>(joint_state_handles_[i].getEffort());
        }

        // leftFootSixAxis
        hardware_interface::ForceTorqueSensorHandle &l_foot_force_torque = force_torque_handles_["leftFootSixAxis"];
        est_robot_state_.force_torque.l_foot_force_z = static_cast<float>(l_foot_force_torque.getForce()[2]);
        est_robot_state_.force_torque.l_foot_torque_x = static_cast<float>(l_foot_force_torque.getTorque()[0]);
        est_robot_state_.force_torque.l_foot_torque_y = static_cast<float>(l_foot_force_torque.getTorque()[1]);

        // rightFootSixAxis
        hardware_interface::ForceTorqueSensorHandle &r_foot_force_torque = force_torque_handles_["rightFootSixAxis"];
        est_robot_state_.force_torque.r_foot_force_z = static_cast<float>(r_foot_force_torque.getForce()[2]);
        est_robot_state_.force_torque.r_foot_torque_x = static_cast<float>(r_foot_force_torque.getTorque()[0]);
        est_robot_state_.force_torque.r_foot_torque_y = static_cast<float>(r_foot_force_torque.getTorque()[1]);

        lcm_->publish("EST_ROBOT_STATE", &est_robot_state_);
    }

    void JointStatePublisher::publishCoreRobotState(int64_t utime) {
        core_robot_state_.utime = utime;

        for (unsigned int i = 0; i < number_of_joint_interfaces_; i++) {
            core_robot_state_.joint_position[i] = static_cast<float>(joint_state_handles_[i].getPosition());
            core_robot_state_.joint_velocity[i] = static_cast<float>(joint_state_handles_[i].getVelocity());
            core_robot_state_.joint_effort[i] = static_cast<float>(joint_state_handles_[i].getEffort());
        }

        lcm_->publish(core_robot_state_channel_.c_str(), &core_robot_state_);
    }

    void JointStatePublisher::publishIMUReadings(int64_t utime) {
        for (auto it = imu_sensor_handles_.begin(); it != imu_sensor_handles_.end(); it++) {
            bot_core::ins_t lcm_imu_msg;
            std::ostringstream imu_channel;
            imu_channel << "IMU_" << it->first;
            lcm_imu_msg.utime = utime;

            lcm_imu_msg.quat[0] = it->second.getOrientation()[0];
            lcm_imu_msg.quat[1] = it->second.getOrientation()[1];
            lcm_imu_msg.quat[2] = it->second.getOrientation()[2];
            lcm_imu_msg.quat[3] = it->second.getOrientation()[3];

            lcm_imu_msg.gyro[0] = it->second.getAngularVelocity()[0];
            lcm_imu_msg.gyro[1] = it->second.getAngularVelocity()[1];
            lcm_imu_msg.gyro[2] = it->second.getAngularVelocity()[2];

            lcm_imu_msg.accel[0] = it->second.getLinearAcceleration()[0];
            lcm_imu_msg.accel[1] = it->second.getLinearAcceleration()[1];
            lcm_imu_msg.accel[2] = it->second.getLinearAcceleration()[2];

            lcm_imu_msg.mag[0] = 0.0;
            lcm_imu_msg.mag[1] = 0.0;
            lcm_imu_msg.mag[2] = 0.0;

            lcm_imu_msg.pressure = 0.0;
            lcm_imu_msg.rel_alt = 0.0;

            lcm_->publish(imu_channel.str(), &lcm_imu_msg);
        }
    }

    void JointStatePublisher::publishForceTorqueReadings(int64_t utime) {
        bot_core::six_axis_force_torque_array_t lcm_ft_array_msg;
        lcm_ft_array_msg.utime = utime;
        lcm_ft_array_msg.num_sensors = 2;
        lcm_ft_array_msg.names.resize(2);
        lcm_ft_array_msg.sensors.resize(2);

        // l_foot
        std::vector<double>& taredValuesLeft = this->forceTorqueTaredValueMap["leftFootSixAxis"];
        lcm_ft_array_msg.sensors[0].utime = utime;
        lcm_ft_array_msg.sensors[0].force[0] = force_torque_handles_["leftFootSixAxis"].getForce()[0] - taredValuesLeft[0];
        lcm_ft_array_msg.sensors[0].force[1] = force_torque_handles_["leftFootSixAxis"].getForce()[1] - taredValuesLeft[1];
        lcm_ft_array_msg.sensors[0].force[2] = force_torque_handles_["leftFootSixAxis"].getForce()[2] - taredValuesLeft[2];
        lcm_ft_array_msg.sensors[0].moment[0] = force_torque_handles_["leftFootSixAxis"].getTorque()[0] - taredValuesLeft[3];
        lcm_ft_array_msg.sensors[0].moment[1] = force_torque_handles_["leftFootSixAxis"].getTorque()[1] - taredValuesLeft[4];
        lcm_ft_array_msg.sensors[0].moment[2] = force_torque_handles_["leftFootSixAxis"].getTorque()[2] - taredValuesLeft[5];
        lcm_ft_array_msg.names[0] = "l_foot";

        // r_foot
        std::vector<double>& taredValuesRight = this->forceTorqueTaredValueMap["rightFootSixAxis"];
        lcm_ft_array_msg.sensors[1].utime = utime;
        lcm_ft_array_msg.sensors[1].force[0] = force_torque_handles_["rightFootSixAxis"].getForce()[0] - taredValuesRight[0];
        lcm_ft_array_msg.sensors[1].force[1] = force_torque_handles_["rightFootSixAxis"].getForce()[1] - taredValuesRight[1];
        lcm_ft_array_msg.sensors[1].force[2] = force_torque_handles_["rightFootSixAxis"].getForce()[2] - taredValuesRight[2];
        lcm_ft_array_msg.sensors[1].moment[0] = force_torque_handles_["rightFootSixAxis"].getTorque()[0] - taredValuesRight[3];
        lcm_ft_array_msg.sensors[1].moment[1] = force_torque_handles_["rightFootSixAxis"].getTorque()[1] - taredValuesRight[4];
        lcm_ft_array_msg.sensors[1].moment[2] = force_torque_handles_["rightFootSixAxis"].getTorque()[2] - taredValuesRight[5];
        lcm_ft_array_msg.names[1] = "r_foot";


        lcm_->publish("FORCE_TORQUE", &lcm_ft_array_msg); 
    }


    void JointStatePublisher::updateForceTorqueTareValues(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                                          const drc::string_t* msg) {


        std::vector<std::string> ftNameVec;

        if (msg->data == "both"){
            ftNameVec.push_back("leftFootSixAxis");
            ftNameVec.push_back("rightFootSixAxis");
            std::cout << "taring LEFT and RIGHT 6-axis force torque sensors" << std::endl;
        } else if (msg->data == "left"){
            ftNameVec.push_back("leftFootSixAxis");
            std::cout << "taring LEFT 6-axis force torque sensors" << std::endl;
        } else if (msg->data == "right"){
            ftNameVec.push_back("rightFootSixAxis");
            std::cout << "taring RIGHT 6-axis force torque sensors" << std::endl;
        } else{
            std::cout << "got an improperly formatted tare msg, doing nothing" << std::endl;
        }

        for(int i = 0; i < ftNameVec.size(); i++){
            std::string ftName = ftNameVec[i];
            hardware_interface::ForceTorqueSensorHandle & FTHandle = force_torque_handles_[ftName];
            std::vector<double>& ftTaredValue = forceTorqueTaredValueMap[ftName];
            ftTaredValue[0] = FTHandle.getForce()[0];
            ftTaredValue[1] = FTHandle.getForce()[1];
            ftTaredValue[2] = FTHandle.getForce()[2];

            ftTaredValue[3] = FTHandle.getTorque()[0];
            ftTaredValue[4] = FTHandle.getTorque()[1];
            ftTaredValue[5] = FTHandle.getTorque()[2];
        }
    }


    void JointStatePublisher::publishForceTorqueTareValues(int64_t utime) {
        bot_core::six_axis_force_torque_array_t msg;
        msg.utime = utime;
        msg.num_sensors = 2;
        msg.sensors.resize(2);


        std::vector<std::string> ftNameVec = {"leftFootSixAxis", "rightFootSixAxis"};
        std::vector<std::string> ftNameMsgVec = {"l_foot", "r_foot"};

        msg.names = ftNameMsgVec;

        for(int i = 0; i < ftNameVec.size(); i++){
            std::string ftName = ftNameVec[i];
            std::vector<double>& ftTaredValue = forceTorqueTaredValueMap[ftName];
            msg.sensors[i].force[0] = ftTaredValue[0];
            msg.sensors[i].force[1] = ftTaredValue[1];
            msg.sensors[i].force[2] = ftTaredValue[2];

            msg.sensors[i].moment[0] = ftTaredValue[3];
            msg.sensors[i].moment[1] = ftTaredValue[4];
            msg.sensors[i].moment[2] = ftTaredValue[5];

        }

        lcm_->publish("FOOT_SENSOR_TARED_VAL", &msg);

    }


    void JointStatePublisher::initializeLCMSubscriptions() {
        lcm_->subscribe("TARE_FOOT_SENSORS", &JointStatePublisher::updateForceTorqueTareValues, this);
    }


    void JointStatePublisher::stopping(const ros::Time &time) { }

}  // namespace valkyrie_translator

PLUGINLIB_EXPORT_CLASS(valkyrie_translator::JointStatePublisher, controller_interface::ControllerBase)
