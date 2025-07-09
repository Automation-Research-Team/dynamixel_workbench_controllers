/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.hpp>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

namespace dynamixel_workbench_controller
{
/************************************************************************
*  class DynamixelController						*
************************************************************************/
class DynamixelController : public rclcpp::Node
{
  private:
    using twist_t		= geometry_msgs::msg::Twist;
    using twist_cp		= twist_t::UniquePtr;
    using trajectory_t		= trajectory_msgs::msg::JointTrajectory;
    using trajectory_cp		= trajectory_t::UniquePtr;
    using joint_state_t		= sensor_msgs::JointState;

    using dynamixel_state_t	= dynamixel_workbench_msgs::msg::
					DynamixelState;
    using dynamixel_states_t	= dynamixel_workbench_msgs::msg::
					DynamixelStateList;
    using dynamixel_states_cp	= dynamixel_states_t::UniquePtr;
    using dynamixel_command_t	= dynamixel_workbench_msgs::srv::
					DynamixelCommand;

    template <class MSG>
    using publisher_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using subscription_p	= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using server_p		= typename rclcpp::Server<SRV>::SharedPtr;


    struct ItemValue
    {
	std::string	item_name;
	int32_t		value;
    };

  public:
    DynamixelController(const rclcpp::NodeOptions& options);

  private:
    bool	initWorkbench(const std::string& port_name,
			      const uint32_t baud_rate)			;
    bool	getDynamixelsInfo(const std::string& yaml_file)		;
    bool	loadDynamixels()					;
    bool	initDynamixels()					;
    bool	initControlItems()					;
    bool	initSDKHandlers()					;
    bool	getPresentPosition(std::vector<std::string> dxl_name)	;

    double	getReadPeriod()		const	{ return read_period_; }
    double	getWritePeriod()	const	{ return write_period_;}
    double	getPublishPeriod()	const	{ return pub_period_; }

    void	readCallback()						;
    void	writeCallback()						;
    void	publishCallback()					;

    void	commandVelocityCallback(const twist_cp& twist)		;
    void	trajectoryMsgCallback(const trajectory_cp& trajectory)	;
    bool	dynamixelCommandMsgCallback(
		    const dynamixel_command_t::Request::SharedPtr& req,
		    dynamixel_command_t::Response::SharedPtr& res)	;

  private:
    const bool					is_joint_state_topic_;
    const bool					is_cmd_vel_topic_;
    const bool					use_moveit_;

  // ROS Topic Publisher
    const publisher_p<dynamixel_states_t>	dynamixel_state_list_pub_;
    const publisher_p<joint_state_t>		joint_states_pub_;
    const subscription_p<twist_t>		cmd_vel_sub_;
    const server_p<dynamixel_command_t>		trajectory_sub_;


  // Dynamixel Workbench Parameters
    std::unique_ptr<DynamixelWorkbench>			dxl_wb_;

    std::map<std::string, uint8_t>			dynamixel_;
    std::map<std::string, const ControlItem*>		control_items_;
    std::vector<std::pair<std::string, ItemValue>>	dynamixel_info_;
    dynamixel_workbench_msgs::DynamixelStateList	dynamixel_state_list_;
    sensor_msgs::JointState				joint_state_msg_;
    std::vector<WayPoint>				pre_goal_;


    double						wheel_separation_;
    double						wheel_radius_;

    std::shared_ptr<JointTrajectory>			jnt_tra_;
    std::shared_ptr<trajectory_msgs::JointTrajectory>	jnt_tra_msg_;

    double						read_period_;
    double						write_period_;
    double						pub_period_;

    bool						is_moving_;
};
}	// namespace dynamixel_workbench_controller
