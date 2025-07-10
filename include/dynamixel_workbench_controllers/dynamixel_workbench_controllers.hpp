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
    using twist_t		= geometry_msgs::msg::Twist;
    using twist_cp		= twist_t::UniquePtr;
    using trajectory_t		= trajectory_msgs::msg::JointTrajectory;
    using trajectory_cp		= trajectory_t::UniquePtr;
    using trajectory_point_t	= trajectory_msgs::msg::JointTrajectoryPoint;
    using joint_state_t		= sensor_msgs::msg::JointState;

    using dynamixel_state_t	= dynamixel_workbench_msgs::msg::
					DynamixelState;
    using dynamixel_states_t	= dynamixel_workbench_msgs::msg::
					DynamixelStateList;
    using dynamixel_states_cp	= dynamixel_states_t::UniquePtr;
    using dynamixel_command_t	= dynamixel_workbench_msgs::srv::
					DynamixelCommand;
    using dynamixel_command_req	= dynamixel_command_t::Request::SharedPtr;
    using dynamixel_command_res	= dynamixel_command_t::Response::SharedPtr;

    template <class MSG>
    using publisher_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using subscription_p	= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using service_p		= typename rclcpp::Service<SRV>::SharedPtr;
    using timer_p		= rclcpp::TimerBase::SharedPtr;

    struct ItemValue
    {
	std::string	dxl_name;
	std::string	item_name;
	int32_t		value;
    };

  public:
    DynamixelController(const rclcpp::NodeOptions& options);

  private:
    void	initWorkbench(const std::string& port_name,
			      const uint32_t baud_rate)			;
    void	initDynamixels(const std::string& yaml_file)		;
    void	initControlItems()					;
    void	initSDKHandlers()					;

    void	dynamixelCommandCallback(const dynamixel_command_req req,
					 dynamixel_command_res res)	;
    void	twistCallback(const twist_cp& twist)			;
    void	trajectoryCallback(const trajectory_cp& trajectory)	;

    void	readCallback()						;
    void	writeCallback()						;
    void	publishCallback()					;

    std::vector<WayPoint>
		getWayPoints(const std::vector<std::string>& joint_names);

  private:
  // ROS publisher/subscriber/service
    const publisher_p<dynamixel_states_t>	dxl_states_pub_;
    const publisher_p<joint_state_t>		joint_state_pub_;
    const subscription_p<twist_t>		twist_sub_;
    const subscription_p<trajectory_t>		trajectory_sub_;
    const service_p<dynamixel_command_t>	dxl_command_srv_;

  // Dynamixel Workbench
    DynamixelWorkbench				dxl_wb_;
    std::map<std::string, uint8_t>		dxl_ids_;
    std::map<std::string, const ControlItem*>	control_items_;
    dynamixel_states_t				dxl_states_;

    double					wheel_separation_;
    double					wheel_radius_;
    const bool					use_moveit_;

    trajectory_t				trajectory_;
    size_t					point_cnt_;
    size_t					position_cnt_;
    bool					is_moving_;

    const double				write_period_;
    const timer_p				read_timer_;
    const timer_p				write_timer_;
    const timer_p				publish_timer_;
};
}	// namespace dynamixel_workbench_controllers
