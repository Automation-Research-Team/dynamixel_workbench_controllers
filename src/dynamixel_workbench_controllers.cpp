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

#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <dynamixel_workbench_controllers/dynamixel_workbench_controllers.hpp>

namespace dynamixel_workbench_controllers
{
/************************************************************************
*  class DynamixelController						*
************************************************************************/
DynamixelController::DynamixelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("dynamixel_workbench_controllers", options),
     dxl_states_pub_(create_publisher<dynamixel_states_t>("~/dynamixel_state",
							  100)),
     joint_state_pub_(ddynamic_reconfigure2::declare_read_only_parameter<bool>(
			  this, "use_joint_states_topic", false) ?
		      create_publisher<joint_state_t>("joint_states", 100) :
		      nullptr),
     twist_sub_(create_subscription<twist_t>(
		    "~/cmd_vel", 100,
		    std::bind(&DynamixelController::twistCallback,
			      this, std::placeholders::_1))),
     trajectory_sub_(create_subscription<trajectory_t>(
			 "~/joint_trajectory", 100,
			 std::bind(&DynamixelController::trajectoryCallback,
				   this, std::placeholders::_1))),
     dxl_command_srv_(
	 create_service<dynamixel_command_t>(
	     "~/dynamixel_command",
	     std::bind(&DynamixelController::dynamixelCommandCallback,
		       this, std::placeholders::_1, std::placeholders::_2))),

     dxl_wb_(),
     dxl_ids_(),
     control_items_(),
     dxl_states_(),

     wheel_separation_(ddynamic_reconfigure2::
		       declare_read_only_parameter<double>(
			   this,
			   "mobile_robot_config.separation_between_wheels",
			   0.0)),
     wheel_radius_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		       this, "mobile_robot_config.radius_of_wheel", 0.0)),
     use_moveit_(ddynamic_reconfigure2::declare_read_only_parameter<bool>(
		     this, "use_moveit", false)),

     trajectory_(),
     point_cnt_(0),
     position_cnt_(0),
     is_moving_(false),

     write_period_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		       this, "dxl_write_period", 0.010)),
     read_timer_(create_wall_timer(
		     std::chrono::duration<double>(
			 ddynamic_reconfigure2::
			 declare_read_only_parameter<double>(
			     this, "dxl_read_period", 0.010)),
		     std::bind(&DynamixelController::readCallback, this))),
     write_timer_(create_wall_timer(
		      std::chrono::duration<double>(write_period_),
		      std::bind(&DynamixelController::writeCallback, this))),
     publish_timer_(create_wall_timer(
			std::chrono::duration<double>(
			    ddynamic_reconfigure2::
			    declare_read_only_parameter<double>(
				this, "publish_period", 0.010)),
			std::bind(&DynamixelController::publishCallback,
				  this)))
{
    try
    {
	initWorkbench(ddynamic_reconfigure2::
		      declare_read_only_parameter<std::string>(
			  this, "port_name", "/dev/ttyUSB1"),
		      ddynamic_reconfigure2::declare_read_only_parameter<int>(
			  this, "baud_rate", 1000000));
	initDynamixels(ddynamic_reconfigure2::
		       declare_read_only_parameter<std::string>(
			   this, "dynamixel_info", ""));
	initSDKHandlers();
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Failed initialization: " << err.what());
	return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Started");
}

void
DynamixelController::initWorkbench(const std::string& port_name,
				   const uint32_t baud_rate)
{
    if (const char* log; !dxl_wb_.init(port_name.c_str(), baud_rate, &log))
	throw std::runtime_error(log);
}

void
DynamixelController::initDynamixels(const std::string& yaml_file)
{
  // Get Dynamixel IDs and list up setting items for each Dynamixel.
    std::vector<ItemValue>	item_values;
    YAML::Node			dynamixel = YAML::LoadFile(yaml_file);

    for (YAML::const_iterator it_file = dynamixel.begin();
	 it_file != dynamixel.end(); ++it_file)
    {
	if (const auto name = it_file->first.as<std::string>(); name.size())
	{
	    YAML::Node	item = dynamixel[name];
	    for (auto it_item = item.begin(); it_item != item.end(); ++it_item)
	    {
		const auto	item_name = it_item->first.as<std::string>();
		const auto	value	  = it_item->second.as<int>();

		if (item_name == "ID")
		    dxl_ids_[name] = uint8_t(value);

		item_values.push_back({name, item_name, value});
	    }
	}
    }

  // Check existence of each Dynamixel with spedified ID.
    for (const auto& dxl_id : dxl_ids_)
    {
	uint16_t	model_number = 0;
	if (const char*	log; !dxl_wb_.ping(dxl_id.second, &model_number, &log))
	    throw std::runtime_error(std::string(log) +
				     ": Can't find Dynamixel ID " +
				     std::to_string(dxl_id.second));

	RCLCPP_INFO_STREAM(get_logger(),
			   "Name : " << dxl_id.first
			   << ", ID : " << dxl_id.second
			   << ", Model Number : " << model_number);
    }

  // Set values specified in YAML to items for each Dynamixel.
    for (const auto& dxl_id : dxl_ids_)
    {
	dxl_wb_.torqueOff(dxl_id.second);

	for (const auto& item_value : item_values)
	{
	    if (const char* log;
		dxl_id.first == item_value.dxl_name	&&
		item_value.item_name != "ID"		&&
		item_value.item_name != "Baud_Rate"	&&
		!dxl_wb_.itemWrite(dxl_id.second, item_value.item_name.c_str(),
				   item_value.value, &log))
	    {
		throw std::runtime_error(std::string(log) +
					 ": Failed to write value[" +
					 std::to_string(item_value.value) +
					 "] on items[" +
					 item_value.item_name +
					 "] to Dynamixel[Name : " +
					 dxl_id.first + ", ID : " +
					 std::to_string(dxl_id.second) + ']');
	    }
	}

	dxl_wb_.torqueOn(dxl_id.second);
    }
}

void
DynamixelController::initControlItems()
{
    const auto	dxl_id = dxl_ids_.begin()->second;
    const auto  goal_position = dxl_wb_.getItemInfo(dxl_id, "Goal_Position");
    if (!goal_position)
	throw std::runtime_error("Failed to get Goal_Position");
    control_items_["Goal_Position"] = goal_position;

    auto	goal_velocity = dxl_wb_.getItemInfo(dxl_id, "Goal_Velocity");
    if (!goal_velocity)
    {
	goal_velocity = dxl_wb_.getItemInfo(dxl_id, "Moving_Speed");
	if (!goal_velocity)
	    throw std::runtime_error("Failed to get Goal_Velocity");
    }
    control_items_["Goal_Velocity"] = goal_velocity;

    const auto	present_position = dxl_wb_.getItemInfo(dxl_id,
						       "Present_Position");
    if (present_position == NULL)
	throw std::runtime_error("Failed to get Present_Position");
    control_items_["Present_Position"] = present_position;

    auto	present_velocity = dxl_wb_.getItemInfo(dxl_id,
						       "Present_Velocity");
    if (present_velocity == NULL)
    {
	present_velocity = dxl_wb_.getItemInfo(dxl_id, "Present_Speed");
	if (present_velocity == NULL)
	    throw std::runtime_error("Failed to get Present_Velocity");
    }
    control_items_["Present_Velocity"] = present_velocity;

    auto	present_current = dxl_wb_.getItemInfo(dxl_id,
						      "Present_Current");
    if (present_current == NULL)
    {
	present_current = dxl_wb_.getItemInfo(dxl_id, "Present_Load");
	if (present_current == NULL)
	    throw std::runtime_error("Failed to get Present_Current");
    }
    control_items_["Present_Current"] = present_current;
}

void
DynamixelController::initSDKHandlers()
{
    const char*	log = NULL;

    if (!dxl_wb_.addSyncWriteHandler(
	    control_items_["Goal_Position"]->address,
	    control_items_["Goal_Position"]->data_length, &log))
	throw std::runtime_error(log);
    else
	RCLCPP_INFO_STREAM(get_logger(), log);

    if (!dxl_wb_.addSyncWriteHandler(
	    control_items_["Goal_Velocity"]->address,
	    control_items_["Goal_Velocity"]->data_length, &log))
	throw std::runtime_error(log);
    else
	RCLCPP_INFO_STREAM(get_logger(), log);

    if (dxl_wb_.getProtocolVersion() == 2.0f)
    {
	uint16_t start_address
			= std::min(control_items_["Present_Position"]->address,
				   control_items_["Present_Current"]->address);

      /*
	As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
      */
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
	uint16_t read_length = control_items_["Present_Position"]->data_length
			     + control_items_["Present_Velocity"]->data_length
			     + control_items_["Present_Current"]->data_length
			     + 2;

	if (!dxl_wb_.addSyncReadHandler(start_address, read_length, &log))
	    throw std::runtime_error(log);
    }
}

/*!
 *  Callback for service dynamixel_workbench_msgs::srv::DynamixelCommand
 */
void
DynamixelController::dynamixelCommandCallback(
    const dynamixel_command_req req, dynamixel_command_res res)
{
    if (const char* log;
	!dxl_wb_.itemWrite(uint8_t(req->id), req->addr_name.c_str(),
			    int32_t(req->value), &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    log << "Failed to write value[" << req->value
			    << "] on items[" << req->addr_name
			    << "] to Dynamixel[ID : " << req->id << ']');
	res->comm_result = false;
    }
    else
	res->comm_result = true;
}

/*!
 *  Transform the subscribed twist command to the left/right wheel velocities
 *  and send them to Dynamixels.
 */
void
DynamixelController::twistCallback(const twist_cp& twist)
{
    std::vector<uint8_t>	id_array;
    float			rpm = 0.0;
    for (auto const& dxl_id : dxl_ids_)
    {
	const auto*	modelInfo = dxl_wb_.getModelInfo(dxl_id.second);
	rpm = modelInfo->rpm;
	id_array.push_back(dxl_id.second);
    }

  //  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
  //       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

    const uint8_t		LEFT  = 0;
    const uint8_t		RIGHT = 1;
    std::vector<double>		wheel_velocity(dxl_ids_.size());
    std::vector<int32_t>	dynamixel_velocity(dxl_ids_.size());

    double	velocity_constant_value = 1.0/(wheel_radius_ * rpm * 0.10472);


    wheel_velocity[LEFT]  = twist->linear.x
			  - (twist->angular.z * wheel_separation_ / 2);
    wheel_velocity[RIGHT] = twist->linear.x
			  + (twist->angular.z * wheel_separation_ / 2);

    if (dxl_wb_.getProtocolVersion() == 2.0f)
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
    else if (dxl_wb_.getProtocolVersion() == 1.0f)
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

    if (const char* log;
	!dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
			    id_array.data(), id_array.size(),
			    dynamixel_velocity.data(), 1, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
    }
}

/*!
 *  Transform the subscribed trajectory command to the waypoints left/right wheel velocities
 *  and send them to Dynamixels.
 */
void
DynamixelController::trajectoryCallback(const trajectory_cp& trajectory)
{
    if (is_moving_)
    {
	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
			     "Dynamixel is moving");
	return;
    }

    if (use_moveit_)
    {
	trajectory_ = *trajectory;
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
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(), err.what());
    }

    RCLCPP_INFO_STREAM(get_logger(), "Succeeded to get joint trajectory!");
    is_moving_ = true;
}

/*!
 *  Read Dynamixels' states and store them in this->dxl_states_.
 */
void
DynamixelController::readCallback()
{
    if (!is_moving_)
	return;

#ifdef DEBUG
    static double	priv_read_secs = get_clock()->now();
#endif

    if (dxl_wb_.getProtocolVersion() == 2.0f)
    {
	std::vector<uint8_t>	id_array;
	for (const auto& dxl_id : dxl_ids_)
	    id_array.push_back(dxl_id.second);

	std::vector<int32_t>	currents(id_array.size());
	std::vector<int32_t>	velocities(id_array.size());
	std::vector<int32_t>	positions(id_array.size());

	if (const char* log;
	    !dxl_wb_.syncRead(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(), &log)			||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		control_items_["Present_Current"]->address,
		control_items_["Present_Current"]->data_length,
		currents.data(), &log)				||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		control_items_["Present_Velocity"]->address,
		control_items_["Present_Velocity"]->data_length,
		velocities.data(), &log)				||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		control_items_["Present_Position"]->address,
		control_items_["Present_Position"]->data_length,
		positions.data(), &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(), log);
	}

	dxl_states_.dynamixel_state.clear();

	for (size_t i = 0; i < id_array.size(); ++i)
	{
	    dynamixel_state_t	dynamixel_state;
	    dynamixel_state.present_position = positions[i];
	    dynamixel_state.present_velocity = velocities[i];
	    dynamixel_state.present_current  = currents[i];

	    dxl_states_.dynamixel_state.push_back(dynamixel_state);
	}
    }
    else if (dxl_wb_.getProtocolVersion() == 1.0f)
    {
	const auto	length_of_data
			    = control_items_["Present_Position"]->data_length
			    + control_items_["Present_Velocity"]->data_length
			    + control_items_["Present_Current"]->data_length;
	std::vector<uint32_t>	all_data(length_of_data);

	dxl_states_.dynamixel_state.clear();

	for (const auto& dxl_id : dxl_ids_)
	{
	    if (const char* log;
		!dxl_wb_.readRegister(
		    dxl_id.second,
		    control_items_["Present_Position"]->address,
		    length_of_data, all_data.data(), &log))
	    {
		RCLCPP_ERROR_STREAM(get_logger(), log);
	    }

	    dynamixel_state_t	dynamixel_state;
	    dynamixel_state.present_position = DXL_MAKEWORD(all_data[0],
							    all_data[1]);
	    dynamixel_state.present_velocity = DXL_MAKEWORD(all_data[2],
							    all_data[3]);
	    dynamixel_state.present_current  = DXL_MAKEWORD(all_data[4],
							    all_data[5]);

	    dxl_states_.dynamixel_state.push_back(dynamixel_state);
	}
    }

#ifdef DEBUG
    RCLCPP_WARN_STREAM(get_logger(),
		       "[readCallback] diff_secs : "
		       << (get_clock()->now() - priv_read_secs).seconds();
    priv_read_secs = get_clock->now();
#endif
}

/*!
 *  Select the oldest but yet written point in the trajectory subscribed
 *  by trajectoryCallback() and send it to the Dynamixels.
 */
void
DynamixelController::writeCallback()
{
    if (!is_moving_)
	return;

#ifdef DEBUG
    static auto	priv_pub_secs = get_clock()->now();
#endif

    std::vector<uint8_t>	id_array;
    for (const auto& joint_name : trajectory_.joint_names)
	id_array.push_back(dxl_ids_[joint_name]);

    std::vector<int32_t>	positions;
    for (size_t i = 0; i < id_array.size(); ++i)
	positions.push_back(
	    dxl_wb_.convertRadian2Value(
		id_array[i],
		trajectory_.points[point_cnt_].positions.at(i)));

    if (const char* log;
	!dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
			   id_array.data(), id_array.size(),
			   positions.data(), 1, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
    }

    ++position_cnt_;

    if (position_cnt_ >= trajectory_.points[point_cnt_].positions.size())
    {
	point_cnt_++;
	position_cnt_ = 0;
	if (point_cnt_ >= trajectory_.points.size())
	{
	    point_cnt_    = 0;
	    position_cnt_ = 0;
	    is_moving_    = false;

	    RCLCPP_INFO_STREAM(get_logger(), "Complete Execution");
	}
    }

#ifdef DEBUG
    RCLCPP_WARN_STREAM(get_logger(),
		       "[writeCallback] diff_secs : "
		       << (get_clock()->now() - priv_pub_secs).seconds());
    priv_pub_secs = get_clock()->now();
#endif
}

/*!
 *  Publish Dynamixels' states and joint state as ROS topics.
 */
void
DynamixelController::publishCallback()
{
#ifdef DEBUG
    static double	priv_pub_secs = get_clock()->now();
#endif
  // Publish Dynamixels' states.
    dxl_states_pub_->publish(dxl_states_);

  // Publish joint state.
    if (joint_state_pub_)
    {
	joint_state_t	joint_state;
	joint_state.header.stamp = get_clock()->now();

	auto	dxl_state = dxl_states_.dynamixel_state.cbegin();
	for (const auto& dxl_id : dxl_ids_)
	{
	    const auto	position = dxl_wb_.convertValue2Radian(
					dxl_id.second,
					int32_t(dxl_state->present_position));
	    const auto	velocity = dxl_wb_.convertValue2Velocity(
					dxl_id.second,
					int32_t(dxl_state->present_velocity));
	    const auto	effort	 = (dxl_wb_.getProtocolVersion() == 2.0f &&
				    strcmp(dxl_wb_.getModelName(dxl_id.second),
					   "XL-320") ?
				    dxl_wb_.convertValue2Load(
					int16_t(dxl_state->present_current)) :
				    dxl_wb_.convertValue2Current(
					int16_t(dxl_state->present_current)));

	    joint_state.name.push_back(dxl_id.first);
	    joint_state.position.push_back(position);
	    joint_state.velocity.push_back(velocity);
	    joint_state.effort.push_back(effort);

	    ++dxl_state;
	}

	joint_state_pub_->publish(joint_state);
    }

#ifdef DEBUG
    RCLCPP_WARN_STREAM(get_logger(), "[publishCallback] diff_secs : "
		       << (get_clock()->now() - priv_pub_secs).seconds());
    priv_pub_secs = get_clock()->now();
#endif
}

std::vector<WayPoint>
DynamixelController::getWayPoints(const std::vector<std::string>& joint_names)
{
    std::vector<uint8_t>	id_array;
    for (const auto& joint_name : joint_names)
	id_array.push_back(dxl_ids_[joint_name]);

    std::vector<WayPoint>	way_points;
    if (dxl_wb_.getProtocolVersion() == 2.0f)
    {
	std::vector<int32_t>	positions(id_array.size());
	if (const char* log;
	    !dxl_wb_.syncRead(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(), &log)			||
	    !dxl_wb_.getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		control_items_["Present_Position"]->address,
		control_items_["Present_Position"]->data_length,
		positions.data(), &log))
	{
	    throw std::runtime_error(log);
	}

	for(size_t i = 0; id_array.size(); ++i)
	{
	    WayPoint	wp;
	    wp.position	    = dxl_wb_.convertValue2Radian(id_array[i],
							  positions[i]);
	    wp.velocity	    = 0.0f;
	    wp.acceleration = 0.0f;
	    way_points.push_back(wp);
	}
    }
    else if (dxl_wb_.getProtocolVersion() == 1.0f)
    {
	for (const auto id : id_array)
	{
	    uint32_t position;

	    if (const char* log;
		!dxl_wb_.readRegister(
		    id,
		    control_items_["Present_Position"]->address,
		    control_items_["Present_Position"]->data_length,
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
