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

/* Authors: Taehun Lim (Darby), Toshio Ueshiba (AIST) */

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.hpp>

#include "trajectory_generator.hpp"

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace dynamixel_workbench_controllers
{
/************************************************************************
*  static functions							*
************************************************************************/
static std::string
filepath_from_url(const std::string& url)
{
  // Split the input URL into tokens by the dellimiter '/'.
    std::vector<std::string>	tokens;
    size_t			pos = 0;
    for (size_t epos;
	 (epos = url.find_first_of('/', pos)) != std::string::npos;
	 pos = epos + 1)
	tokens.push_back(url.substr(pos, epos - pos));
    if (pos < url.size())
	tokens.push_back(url.substr(pos));

    if (tokens.size() < 3 || tokens[1] != "")
	throw std::runtime_error("illegal URL: " + url);

    std::filesystem::path	path;
    if (tokens[0] == "package:")
        path = ament_index_cpp::get_package_share_directory(tokens[2]);
    else if (tokens[0] == "file:")
        path = "/";
    else
        throw std::runtime_error("unknown URL scheme: " + tokens[0]);

    for (size_t n = 3; n < tokens.size(); ++n)
	path /= tokens[n];

    return path.string<char>();
}

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
    using twist_t		= geometry_msgs::msg::Twist;
    using trajectory_t		= trajectory_msgs::msg::JointTrajectory;
    using trajectory_point_t	= trajectory_msgs::msg::JointTrajectoryPoint;
    using trajectory_point_iter	= std::vector<trajectory_point_t>::
					const_iterator;
    using joint_state_t		= sensor_msgs::msg::JointState;
    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;
    using timer_p		= rclcpp::TimerBase::SharedPtr;

    template <class MSG>
    using pub_p			= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p			= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class MSG>
    using msg_p			= typename MSG::UniquePtr;
    template <class SRV>
    using srv_p			= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp		= typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p			= typename SRV::Response::SharedPtr;

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
    void	initDynamixels(const std::string& yaml_url)		;
    void	initControlItems()					;

    void	dynamixelCommandCallback(req_cp<dynamixel_command_t> req,
					 res_p<dynamixel_command_t> res);
    void	twistCallback(msg_p<twist_t> twist)			;
    void	trajectoryCallback(msg_p<trajectory_t> trajectory)	;

    void	readDynamixelStatesCallback()				;
    void	writeTrajectoryPointCallback()				;

    std::vector<WayPoint>
		getWayPoints(const std::vector<std::string>& joint_names);

  private:
  // Dynamixel Workbench
    DynamixelWorkbench			dxl_wb_;
    std::map<std::string, uint8_t>	dxl_ids_;
    const ControlItem*			dxl_present_position_;
    const ControlItem*			dxl_present_velocity_;
    const ControlItem*			dxl_present_current_;
    float				dxl_protocol_version_;
    std::mutex				dxl_mtx_;

  // Differential wheel drive stuffs conmmanded by twist tocpic
    double				wheel_separation_;
    double				wheel_radius_;

  // Joint trajectory buffer stuffs commanded by joint trajectory topic
    const bool				use_moveit_;
    trajectory_t			trajectory_;
    trajectory_point_iter		current_point_;
    std::mutex				current_point_mtx_;

  // ROS service, subscribers, publishers and timers
    const callback_group_p		dxl_command_callback_group_;
    const srv_p<dynamixel_command_t>	dxl_command_srv_;
    const sub_p<twist_t>		twist_sub_;
    const sub_p<trajectory_t>		trajectory_sub_;
    const pub_p<dynamixel_states_t>	dxl_states_pub_;
    const pub_p<joint_state_t>		joint_state_pub_;
    const callback_group_p		read_timer_callback_group_;
    const timer_p			read_timer_;
    const double			write_period_;
    const callback_group_p		write_timer_callback_group_;
    const timer_p			write_timer_;
};

DynamixelController::DynamixelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("dynamixel_workbench_controllers", options),
     dxl_wb_(),
     dxl_ids_(),
     dxl_present_position_(nullptr),
     dxl_present_velocity_(nullptr),
     dxl_present_current_(nullptr),
     dxl_protocol_version_(2.0f),
     dxl_mtx_(),

     wheel_separation_(ddynamic_reconfigure2::declare_read_only_parameter(
			   this,
			   "mobile_robot_config.separation_between_wheels",
			   0.0)),
     wheel_radius_(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "mobile_robot_config.radius_of_wheel", 0.0)),

     use_moveit_(ddynamic_reconfigure2::declare_read_only_parameter(
		     this, "use_moveit", false)),
     trajectory_(),
     current_point_(trajectory_.points.end()),
     current_point_mtx_(),

     dxl_command_callback_group_(
	 create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
     dxl_command_srv_(create_service<dynamixel_command_t>(
			  "~/dynamixel_command",
			  std::bind(
			      &DynamixelController::dynamixelCommandCallback,
			      this,
			      std::placeholders::_1, std::placeholders::_2),
			  rclcpp::ServicesQoS(), dxl_command_callback_group_)),
     twist_sub_(wheel_separation_ > 0.0 && wheel_radius_ > 0.0 ?
		create_subscription<twist_t>(
		    "~/cmd_vel", 100,
		    std::bind(&DynamixelController::twistCallback,
			      this, std::placeholders::_1)) :
		nullptr),
     trajectory_sub_(create_subscription<trajectory_t>(
			 "~/joint_trajectory", 100,
			 std::bind(&DynamixelController::trajectoryCallback,
				   this, std::placeholders::_1))),
     dxl_states_pub_(create_publisher<dynamixel_states_t>("~/dynamixel_state",
							  100)),
     joint_state_pub_(ddynamic_reconfigure2::declare_read_only_parameter(
			  this, "use_joint_states_topic", false) ?
		      create_publisher<joint_state_t>("joint_states", 100) :
		      nullptr),

     read_timer_callback_group_(
	 create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
     read_timer_(create_wall_timer(
		     std::chrono::duration<double>(
			 ddynamic_reconfigure2::declare_read_only_parameter(
			     this, "dxl_read_period", 0.010)),
		     std::bind(
			 &DynamixelController::readDynamixelStatesCallback,
			 this),
		     read_timer_callback_group_)),
     write_period_(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "dxl_write_period", 0.010)),
     write_timer_callback_group_(
	 create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
     write_timer_(create_wall_timer(
		      std::chrono::duration<double>(write_period_),
		      std::bind(
			  &DynamixelController::writeTrajectoryPointCallback,
			  this),
		      write_timer_callback_group_))
{
    try
    {
	initWorkbench(ddynamic_reconfigure2::declare_read_only_parameter(
			  this, "usb_port", "/dev/ttyUSB0"),
		      ddynamic_reconfigure2::declare_read_only_parameter(
			  this, "dxl_baud_rate", 1000000));
	initDynamixels(ddynamic_reconfigure2::declare_read_only_parameter(
			   this, "dynamixel_info", ""));
	initControlItems();
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Initialization failed: " << err.what());
	return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Started");
}

void
DynamixelController::initWorkbench(const std::string& port_name,
				   const uint32_t baud_rate)
{
    if (const char* log = nullptr;
	!dxl_wb_.init(port_name.c_str(), baud_rate, &log))
	throw std::runtime_error(log);
}

void
DynamixelController::initDynamixels(const std::string& yaml_url)
{
  // Get Dynamixel IDs and list up setting items for each Dynamixel.
    std::vector<Item>	items;
    YAML::Node		dynamixel = YAML::LoadFile(filepath_from_url(yaml_url));

    for (auto it_file = dynamixel.begin(); it_file != dynamixel.end();
	 ++it_file)
    {
	const auto	dxl_name = it_file->first.as<std::string>();
	if (dxl_name.size())
	{
	    YAML::Node	item = dynamixel[dxl_name];
	    for (auto it_item = item.begin(); it_item != item.end(); ++it_item)
	    {
		const auto	name  = it_item->first.as<std::string>();
		const auto	value = it_item->second.as<int>();

		if (name == "ID")
		    dxl_ids_[dxl_name] = uint8_t(value);

		items.push_back({dxl_name, name, value});
	    }
	}
    }

  // Check existence of each Dynamixel with spedified ID.
    for (const auto& dxl_id : dxl_ids_)
    {
	uint16_t	model_number = 0;
	if (const char*	log = nullptr;
	    !dxl_wb_.ping(dxl_id.second, &model_number, &log))
	    throw std::runtime_error(std::string(log) +
				     ": Can't find Dynamixel ID " +
				     std::to_string(int(dxl_id.second)));

	RCLCPP_INFO_STREAM(get_logger(),
			   "Name: " << dxl_id.first
			   << ", ID: " << int(dxl_id.second)
			   << ", Model Number: " << model_number);
    }

  // Set values specified in YAML to items for each Dynamixel.
    for (const auto& dxl_id : dxl_ids_)
    {
	dxl_wb_.torqueOff(dxl_id.second);

	for (const auto& item : items)
	{
	    if (const char* log = nullptr;
		dxl_id.first == item.dxl_name	&&
		item.name != "ID"		&&
		item.name != "Baud_Rate"	&&
		!dxl_wb_.itemWrite(dxl_id.second, item.name.c_str(),
				   item.value, &log))
	    {
		throw std::runtime_error(std::string(log) +
					 ": Failed to write value[" +
					 std::to_string(item.value) +
					 "] on item[" + item.name +
					 "] to Dynamixel[Name: " +
					 dxl_id.first + ", ID: " +
					 std::to_string(int(dxl_id.second)) +
					 ']');
	    }
	}

	dxl_wb_.torqueOn(dxl_id.second);
    }
}

void
DynamixelController::initControlItems()
{
    const auto	dxl_id = dxl_ids_.begin()->second;
    const char*	log = nullptr;

  // Add a write handler for Goal_Position
    const auto	dxl_goal_position = dxl_wb_.getItemInfo(dxl_id,
							"Goal_Position");
    if (!dxl_goal_position)
	throw std::runtime_error("Failed to get Goal_Position");
    if (!dxl_wb_.addSyncWriteHandler(dxl_goal_position->address,
				     dxl_goal_position->data_length, &log))
	throw std::runtime_error(log);
    else
	RCLCPP_INFO_STREAM(get_logger(), log);

  // Add a write handler for Goal_Velocity
    auto	dxl_goal_velocity = dxl_wb_.getItemInfo(dxl_id,
							"Goal_Velocity");
    if (!dxl_goal_velocity)
    {
	dxl_goal_velocity = dxl_wb_.getItemInfo(dxl_id, "Moving_Speed");
	if (!dxl_goal_velocity)
	    throw std::runtime_error("Failed to get Goal_Velocity");
    }
    if (!dxl_wb_.addSyncWriteHandler(dxl_goal_velocity->address,
				     dxl_goal_velocity->data_length, &log))
	throw std::runtime_error(log);
    else
	RCLCPP_INFO_STREAM(get_logger(), log);

  // Initialize control item for Present_Position
    dxl_present_position_ = dxl_wb_.getItemInfo(dxl_id, "Present_Position");
    if (!dxl_present_position_)
	throw std::runtime_error("Failed to get Present_Position");

  // Initialize control item for Present_Velocity
    dxl_present_velocity_ = dxl_wb_.getItemInfo(dxl_id, "Present_Velocity");
    if (!dxl_present_velocity_)
    {
	dxl_present_velocity_ = dxl_wb_.getItemInfo(dxl_id, "Present_Speed");
	if (!dxl_present_velocity_)
	    throw std::runtime_error("Failed to get Present_Velocity");
    }

  // Initialize control item for Present_Current
    dxl_present_current_ = dxl_wb_.getItemInfo(dxl_id, "Present_Current");
    if (!dxl_present_current_)
    {
	dxl_present_current_ = dxl_wb_.getItemInfo(dxl_id, "Present_Load");
	if (!dxl_present_current_)
	    throw std::runtime_error("Failed to get Present_Current");
    }

    dxl_protocol_version_ = dxl_wb_.getProtocolVersion();

    if (dxl_protocol_version_ == 2.0f)
    {
	const uint16_t start_address = std::min(dxl_present_position_->address,
						dxl_present_current_->address);

      /*
	As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
      */
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
	const uint16_t read_length = dxl_present_position_->data_length
				   + dxl_present_velocity_->data_length
				   + dxl_present_current_->data_length
				   + 2;

	if (!dxl_wb_.addSyncReadHandler(start_address, read_length, &log))
	    throw std::runtime_error(log);
    }
}

/*!
 *  Callback for service dynamixel_workbench_msgs::srv::DynamixelCommand
 */
void
DynamixelController::dynamixelCommandCallback(req_cp<dynamixel_command_t> req,
					      res_p<dynamixel_command_t>  res)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "Received service request to write value[" << req->value
		       << "] to item[" << req->addr_name
		       << "] of Dynamixel[" << int(req->id) << ']');

    const std::lock_guard<std::mutex>	lock(dxl_mtx_);

    if (const char* log = nullptr;
	!(res->comm_result = dxl_wb_.itemWrite(req->id, req->addr_name.c_str(),
					       req->value, &log)))
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Failed to write value[" << req->value
			    << "] to item[" << req->addr_name
			    << "] of Dynamixel[" << int(req->id)
			    << "](" << log << ')');
}

/*!
 *  Convert the subscribed twist command to the left/right wheel velocities
 *  and send them to Dynamixels.
 */
void
DynamixelController::twistCallback(msg_p<twist_t> twist)
{
    std::vector<uint8_t>	id_array;
    float			rpm = 0.0;
    for (const auto& dxl_id : dxl_ids_)
    {
	id_array.push_back(dxl_id.second);
	rpm = dxl_wb_.getModelInfo(dxl_id.second)->rpm;
    }

  //  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
  //       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

    const uint8_t		LEFT  = 0;
    const uint8_t		RIGHT = 1;
    std::vector<double>		wheel_velocity(dxl_ids_.size());
    std::vector<int32_t>	dynamixel_velocity(dxl_ids_.size());

    const double velocity_constant_value = 1.0/(wheel_radius_ * rpm * 0.10472);

    wheel_velocity[LEFT]  = twist->linear.x
			  - (twist->angular.z * wheel_separation_ / 2);
    wheel_velocity[RIGHT] = twist->linear.x
			  + (twist->angular.z * wheel_separation_ / 2);

    if (dxl_protocol_version_ == 2.0f)
    {
	if (strcmp(dxl_wb_.getModelName(id_array[0]), "XL-320") == 0)
	{
	    if (wheel_velocity[LEFT] == 0.0)
		dynamixel_velocity[LEFT] = 0;
	    else if (wheel_velocity[LEFT] < 0.0)
		dynamixel_velocity[LEFT] = ((-1.0) * wheel_velocity[LEFT])
					 * velocity_constant_value + 1023;
	    else if (wheel_velocity[LEFT] > 0.0)
		dynamixel_velocity[LEFT] = (wheel_velocity[LEFT]
					    * velocity_constant_value);

	    if (wheel_velocity[RIGHT] == 0.0)
		dynamixel_velocity[RIGHT] = 0;
	    else if (wheel_velocity[RIGHT] < 0.0)
		dynamixel_velocity[RIGHT] = ((-1.0) * wheel_velocity[RIGHT]
					     * velocity_constant_value) + 1023;
	    else if (wheel_velocity[RIGHT] > 0.0)
		dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT]
					     * velocity_constant_value);
	}
	else
	{
	    dynamixel_velocity[LEFT]  = wheel_velocity[LEFT]
				      * velocity_constant_value;
	    dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT]
				      * velocity_constant_value;
	}
    }
    else if (dxl_protocol_version_ == 1.0f)
    {
	if (wheel_velocity[LEFT] == 0.0)
	    dynamixel_velocity[LEFT] = 0;
	else if (wheel_velocity[LEFT] < 0.0)
	    dynamixel_velocity[LEFT] = ((-1.0) * wheel_velocity[LEFT])
				     * velocity_constant_value + 1023;
	else if (wheel_velocity[LEFT] > 0.0)
	    dynamixel_velocity[LEFT] = (wheel_velocity[LEFT]
					* velocity_constant_value);

	if (wheel_velocity[RIGHT] == 0.0)
	    dynamixel_velocity[RIGHT] = 0;
	else if (wheel_velocity[RIGHT] < 0.0)
	    dynamixel_velocity[RIGHT] = ((-1.0) * wheel_velocity[RIGHT]
					 * velocity_constant_value) + 1023;
	else if (wheel_velocity[RIGHT] > 0.0)
	    dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT]
					 * velocity_constant_value);
    }

    const std::lock_guard<std::mutex>	lock(dxl_mtx_);

    if (const char* log = nullptr;
	!dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
			    id_array.data(), id_array.size(),
			    dynamixel_velocity.data(), 1, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
    }
}

/*!
 *  Store the subscribed trajectory command in this->trajectory_
 */
void
DynamixelController::trajectoryCallback(msg_p<trajectory_t> trajectory)
{
    const std::lock_guard<std::mutex>	lock(current_point_mtx_);

    if (current_point_ != trajectory_.points.end())
    {
	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
			     "Dynamixel is moving");
	return;
    }

    if (use_moveit_)
    {
	trajectory_    = *trajectory;
	current_point_ = trajectory_.points.begin();
	return;
    }

    try
    {
      // Get current joint positions.
	auto	start = getWayPoints(trajectory->joint_names);

	trajectory_.joint_names = trajectory->joint_names;
	trajectory_.points.clear();

	const trajectory_point_t*	prev_point = nullptr;
	for (const auto& point : trajectory->points)
	{
	  // Convert each point in the given trajectory
	  // to the array of way points.
	    std::vector<WayPoint>	goal;
	    for (size_t i = 0; i < point.positions.size(); ++i)
	    {
		WayPoint	wp;
		wp.position	= point.positions.at(i);
		wp.velocity	= (i < point.velocities.size() ?
				   point.velocities.at(i) : 0.0f);
		wp.acceleration = (i < point.accelerations.size() ?
				   point.accelerations.at(i) : 0.0f);
		goal.push_back(wp);
	    }

	    using Duration = rclcpp::Duration;

	    const auto	move_time = (!prev_point ?
				     Duration(point.time_from_start) :
				     Duration(point.time_from_start) -
				     Duration(prev_point->time_from_start))
				   .seconds();

	    JointTrajectory	jnt_tra;
	    jnt_tra.setJointNum(uint8_t(point.positions.size()));
	    jnt_tra.init(move_time, write_period_, start, goal);

	  // Generate trajectory points every write_period_ up to move_time.
	    for (double t = 0.0; t < move_time; t += write_period_)
	    {
		trajectory_point_t	trajectory_point;

		for (const auto& wp : jnt_tra.getJointWayPoint(t))
		{
		    trajectory_point.positions.push_back(wp.position);
		    trajectory_point.velocities.push_back(wp.velocity);
		    trajectory_point.accelerations.push_back(wp.acceleration);
		}

		trajectory_.points.push_back(trajectory_point);
	    }

	    start = goal;
	    prev_point = &point;
	}

	current_point_ = trajectory_.points.begin();
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(), err.what());
    }

    RCLCPP_INFO_STREAM(get_logger(), "Succeeded to get joint trajectory!");
}

/*!
 *  Read Dynamixels' states and store them in this->dxl_states_.
 */
void
DynamixelController::readDynamixelStatesCallback()
{
    // if (current_point_ != trajectory_.points.end())	// Moving?
    // 	return;

  // Read Dynamixels' states and convert them to DynamixelStateList message.
    auto	dxl_states = std::make_unique<dynamixel_states_t>();
    if (dxl_protocol_version_ == 2.0f)
    {
	std::vector<std::string>	name_array;
	std::vector<uint8_t>		id_array;
	for (const auto& dxl_id : dxl_ids_)
	{
	    name_array.push_back(dxl_id.first);
	    id_array.push_back(dxl_id.second);
	}

	std::vector<int32_t>	currents(id_array.size());
	std::vector<int32_t>	velocities(id_array.size());
	std::vector<int32_t>	positions(id_array.size());

	const std::lock_guard<std::mutex>	lock(dxl_mtx_);

	if (const char* log = nullptr;
	    !dxl_wb_.syncRead(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(), &log)			||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		dxl_present_current_->address,
		dxl_present_current_->data_length,
		currents.data(), &log)					||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		dxl_present_velocity_->address,
		dxl_present_velocity_->data_length,
		velocities.data(), &log)				||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		dxl_present_position_->address,
		dxl_present_position_->data_length,
		positions.data(), &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(), log);
	}

	dxl_states->dynamixel_state.clear();

	for (size_t i = 0; i < id_array.size(); ++i)
	{
	    dynamixel_state_t	dynamixel_state;
	    dynamixel_state.name	     = name_array[i];
	    dynamixel_state.id		     = id_array[i];
	    dynamixel_state.present_position = positions[i];
	    dynamixel_state.present_velocity = velocities[i];
	    dynamixel_state.present_current  = currents[i];

	    dxl_states->dynamixel_state.push_back(dynamixel_state);
	}
    }
    else if (dxl_protocol_version_ == 1.0f)
    {
	const uint16_t		length_of_data
				    = dxl_present_position_->data_length
				    + dxl_present_velocity_->data_length
				    + dxl_present_current_->data_length;
	std::vector<uint32_t>	all_data(length_of_data);

	dxl_states->dynamixel_state.clear();

	const std::lock_guard<std::mutex>	lock(dxl_mtx_);

	for (const auto& dxl_id : dxl_ids_)
	{
	    if (const char* log = nullptr;
		!dxl_wb_.readRegister(dxl_id.second,
				      dxl_present_position_->address,
				      length_of_data, all_data.data(), &log))
	    {
		RCLCPP_ERROR_STREAM(get_logger(), log);
	    }

	    dynamixel_state_t	dynamixel_state;
	    dynamixel_state.name	     = dxl_id.first;
	    dynamixel_state.id		     = dxl_id.second;
	    dynamixel_state.present_position = DXL_MAKEWORD(all_data[0],
							    all_data[1]);
	    dynamixel_state.present_velocity = DXL_MAKEWORD(all_data[2],
							    all_data[3]);
	    dynamixel_state.present_current  = DXL_MAKEWORD(all_data[4],
							    all_data[5]);

	    dxl_states->dynamixel_state.push_back(dynamixel_state);
	}
    }

    if (joint_state_pub_)
    {
      // Convert Dynamixels' states to joint state and publish it.
	auto	joint_state = std::make_unique<joint_state_t>();
	joint_state->header.stamp = get_clock()->now();

	auto	dxl_state = dxl_states->dynamixel_state.cbegin();
	for (const auto& dxl_id : dxl_ids_)
	{
	    const auto	position = dxl_wb_.convertValue2Radian(
					dxl_id.second,
					int32_t(dxl_state->present_position));
	    const auto	velocity = dxl_wb_.convertValue2Velocity(
					dxl_id.second,
					int32_t(dxl_state->present_velocity));
	    const auto	effort	 = (dxl_protocol_version_ == 2.0f &&
				    strcmp(dxl_wb_.getModelName(dxl_id.second),
					   "XL-320") ?
				    dxl_wb_.convertValue2Load(
					int16_t(dxl_state->present_current)) :
				    dxl_wb_.convertValue2Current(
					int16_t(dxl_state->present_current)));

	    joint_state->name.push_back(dxl_id.first);
	    joint_state->position.push_back(position);
	    joint_state->velocity.push_back(velocity);
	    joint_state->effort.push_back(effort);

	    ++dxl_state;
	}

	joint_state_pub_->publish(std::move(joint_state));
    }

  // Publish Dynamixels' states.
    dxl_states_pub_->publish(std::move(dxl_states));
}

/*!
 *  Select the oldest but yet written point in the trajectory subscribed
 *  by trajectoryCallback() and send it to the Dynamixels.
 */
void
DynamixelController::writeTrajectoryPointCallback()
{
    const std::lock_guard<std::mutex>	lock(current_point_mtx_);

    if (current_point_ == trajectory_.points.end())	// Not moving?
	return;

    std::vector<uint8_t>	id_array;
    for (const auto& joint_name : trajectory_.joint_names)
	id_array.push_back(dxl_ids_[joint_name]);

    std::vector<int32_t>	positions;
    for (size_t i = 0; i < id_array.size(); ++i)
	positions.push_back(dxl_wb_.convertRadian2Value(
				id_array[i], current_point_->positions.at(i)));

    const std::lock_guard<std::mutex>	dxl_lock(dxl_mtx_);

    if (const char* log = nullptr;
	!dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
			   id_array.data(), id_array.size(),
			   positions.data(), 1, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
    }

    ++current_point_;
}

std::vector<WayPoint>
DynamixelController::getWayPoints(const std::vector<std::string>& joint_names)
{
    std::vector<uint8_t>	id_array;
    for (const auto& joint_name : joint_names)
	id_array.push_back(dxl_ids_[joint_name]);

    std::vector<WayPoint>	way_points;
    if (dxl_protocol_version_ == 2.0f)
    {
	const std::lock_guard<std::mutex>	lock(dxl_mtx_);

	std::vector<int32_t>	positions(id_array.size());
	if (const char* log = nullptr;
	    !dxl_wb_.syncRead(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(), &log)			||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		dxl_present_position_->address,
		dxl_present_position_->data_length,
		positions.data(), &log))
	{
	    throw std::runtime_error(log);
	}

	for (size_t i = 0; id_array.size(); ++i)
	{
	    WayPoint	wp;
	    wp.position	    = dxl_wb_.convertValue2Radian(id_array[i],
							  positions[i]);
	    wp.velocity	    = 0.0f;
	    wp.acceleration = 0.0f;
	    way_points.push_back(wp);
	}
    }
    else if (dxl_protocol_version_ == 1.0f)
    {
	const std::lock_guard<std::mutex>	lock(dxl_mtx_);

	for (const auto id : id_array)
	{
	    uint32_t position;

	    if (const char* log = nullptr;
		!dxl_wb_.readRegister(id,
				      dxl_present_position_->address,
				      dxl_present_position_->data_length,
				      &position, &log))
	    {
		throw std::runtime_error(log);
	    }

	    WayPoint	wp;
	    wp.position	    = dxl_wb_.convertValue2Radian(id, position);
	    wp.velocity	    = 0.0f;
	    wp.acceleration = 0.0f;
	    way_points.push_back(wp);
	}
    }

    return way_points;
}
}	// namespace dynamixel_workbench_controllers

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    dynamixel_workbench_controllers::DynamixelController)
