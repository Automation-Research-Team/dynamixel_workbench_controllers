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

#include <dynamixel_workbench_controllers/trajectory_generator.hpp>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

namespace dynamixel_workbench_controllers
{
/************************************************************************
*  class DynamixelController						*
************************************************************************/
class DynamixelController : public rclcpp::Node
{
  private:
    using dynamixel_state_t	= dynamixel_workbench_msgs::msg::
					DynamixelState;
    using dynamixel_states_t	= dynamixel_workbench_msgs::msg::
					DynamixelStateList;
    using dynamixel_command_t	= dynamixel_workbench_msgs::srv::
					DynamixelCommand;
    using dynamixel_command_req	= dynamixel_command_t::Request::SharedPtr;
    using dynamixel_command_res	= dynamixel_command_t::Response::SharedPtr;

    using twist_t		= geometry_msgs::msg::Twist;
    using twist_p		= twist_t::UniquePtr;
    using trajectory_t		= trajectory_msgs::msg::JointTrajectory;
    using trajectory_p		= trajectory_t::UniquePtr;
    using trajectory_point_t	= trajectory_msgs::msg::JointTrajectoryPoint;
    using trajectory_point_iter	= std::vector<trajectory_point_t>::
					const_iterator;
    using joint_state_t		= sensor_msgs::msg::JointState;

    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;
    template <class MSG>
    using publisher_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using subscription_p	= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using service_p		= typename rclcpp::Service<SRV>::SharedPtr;
    using timer_p		= rclcpp::TimerBase::SharedPtr;

    struct Item
    {
	std::string	dxl_name;	// Dynamixel owning this item
	std::string	name;		// name of this item
	int32_t		value;		// value of this item
    };

  public:
    DynamixelController(const rclcpp::NodeOptions& options);

  private:
    void	initWorkbench(const std::string& port_name,
			      const uint32_t baud_rate)			;
    void	initDynamixels(const std::string& yaml_file)		;
    void	initControlItems()					;

    void	dynamixelCommandCallback(const dynamixel_command_req req,
					 dynamixel_command_res res)	;
    void	twistCallback(const twist_p& twist)			;
    void	trajectoryCallback(const trajectory_p& trajectory)	;

    void	readDynamixelStatesCallback()				;
    void	writeTrajectoryPointCallback()				;

    std::vector<WayPoint>
		getWayPoints(const std::vector<std::string>& joint_names);

  private:
  // Dynamixel Workbench
    DynamixelWorkbench				dxl_wb_;
    std::map<std::string, uint8_t>		dxl_ids_;
    const ControlItem*				dxl_present_position_;
    const ControlItem*				dxl_present_velocity_;
    const ControlItem*				dxl_present_current_;

  // Differential wheel drive stuffs conmmanded by twist tocpic
    double					wheel_separation_;
    double					wheel_radius_;

  // Joint trajectory buffer stuffs commanded by joint trajectory topic
    const bool					use_moveit_;
    trajectory_t				trajectory_;
    trajectory_point_iter			current_point_;
    std::mutex					current_point_mtx_;

  // ROS service, subscribers, publishers and timers
    const callback_group_p			dxl_command_callback_group_;
    const service_p<dynamixel_command_t>	dxl_command_srv_;
    const subscription_p<twist_t>		twist_sub_;
    const subscription_p<trajectory_t>		trajectory_sub_;
    const publisher_p<dynamixel_states_t>	dxl_states_pub_;
    const publisher_p<joint_state_t>		joint_state_pub_;
    const callback_group_p			read_timer_callback_group_;
    const timer_p				read_timer_;
    const double				write_period_;
    const callback_group_p			write_timer_callback_group_;
    const timer_p				write_timer_;
};
}	// namespace dynamixel_workbench_controllers
