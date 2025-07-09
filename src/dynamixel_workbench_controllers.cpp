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
     is_joint_state_topic_(ddynamic_reconfigure2::
			   declare_read_only_parameter<bool>(
			       this, "use_joint_states_topic", true)),
     is_cmd_vel_topic_(ddynamic_reconfigure2::
		       declare_read_only_parameter<bool>(
			   this, "use_cmd_vel_topic", false)),
     use_moveit_(ddynamic_reconfigure2::declare_read_only_parameter<bool>(
		     this, "use_moveit", false)),

     dynamixel_state_list_pub_(create_publisher<dynamixel_states_t>(
				   "~/dynamixel_state", 100)),
     joint_states_pub_(is_joint_state_topic_ ?
		       create_publisher<joint_state_t>("joint_states", 100) :
		       nullptr),
     cmd_vel_sub_(is_cmd_vel_topic ?
		  create_subscription<twist_t>(
		      "~/cmd_vel", 100,
		      std::bind(&DynamixelController::commandVelocityCallback,
				this, std::placeholders::_1)) :
		  nullptr),
     trajectory_sub_(create_subscription<twist_t>(
			 "~/joint_trajectory", 100,
			 std::bind(&DynamixelController::trajectoryMsgCallback,
				   this, std::placeholders::_1))),
     dynamixel_command_server_(
	 create_service<dynamixel_command_t>(
	     "~/dynamixel_command",
	     std::bind(&DynamixelController::dynamixelCommandMsgCallback,
		       this, std::placeholders::_1, std::placeholders::_2))),

     dxl_wb_(new DynamixelWorkbench()),
     dynamixel_(),
     control_items_(),
     dynamixel_info_(),
     dynamixel_state_list_(),
     joint_state_msg_(),
     pre_goal_(),

     wheel_separation_(ddynamic_reconfigure2::
		       declare_read_only_parameter<double>(
			   this,
			   "mobile_robot_config.separation_between_wheels",
			   0.0)),
     wheel_radius_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		       this, "mobile_robot_config.radius_of_wheel", 0.0)),

     jnt_tra_(new JointTrajectory),
     jnt_tra_msg_(new trajectory_msgs::JointTrajectory),

     read_period_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		      this, "dxl_read_period", 0.010)),
     write_period_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		       this, "dxl_write_period", 0.010)),
     pub_period_(ddynamic_reconfigure2::declare_read_only_parameter<double>(
		     this, "publish_period", 0.010)),

     is_moving_(false)
{
    if (!initWorkbench(ddynamic_reconfigure2::
		       declare_read_only_parameter<std::string>(
			   this, "port_name", "/dev/ttyUSB1"),
		       ddynamic_reconfigure2::
		       declare_read_only_parameter<uint32_t>(
			   this, "baud_rate", 1000000))			||
	!getDynamixelsInfo(ddynamic_reconfigure2::
			   declare_read_only_parameter<std::string>(
			       this, "dynamixel_info", ""))		||
	!loadDynamixels()						||
	!initSDKHandlers())
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Failed Initialization");
	return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Started");
}

bool
DynamixelController::initWorkbench(const std::string& port_name,
				   const uint32_t baud_rate)
{
    if (const char* log; !dxl_wb_->init(port_name.c_str(), baud_rate, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
	return false;
    }

    return true;
}

bool
DynamixelController::getDynamixelsInfo(const std::string& yaml_file)
{
    YAML::Node	dynamixel = YAML::LoadFile(yaml_file.c_str());

    if (dynamixel == NULL)
	return false;

    for (YAML::const_iterator it_file = dynamixel.begin();
	 it_file != dynamixel.end(); ++it_file)
    {
	if (const auto name = it_file->first.as<std::string>(); name.size())
	{
	    YAML::Node			item	= dynamixel[name];
	    for (YAML::const_iterator	it_item = item.begin();
		 it_item != item.end(); ++it_item)
	    {
		std::string	item_name = it_item->first.as<std::string>();
		int32_t		value	  = it_item->second.as<int32_t>();

		if (item_name == "ID")
		    dynamixel_[name] = uint8_t(value);

		ItemValue	item_value = {item_name, value};
		std::pair<std::string, ItemValue>	info(name, item_value);

		dynamixel_info_.push_back(info);
	    }
	}
    }

    return true;
}

bool
DynamixelController::loadDynamixels()
{
    for (const auto& dxl : dynamixel_)
    {
	uint16_t	model_number = 0;
	if (const char*	log;
	    !dxl_wb_->ping(uint8_t(dxl.second), &model_number, &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(),
				log << ": Can't find Dynamixel ID "
				    << dxl.second);
	    return false;
	}

	RCLCPP_INFO_STREAM(get_logger(),
			   "Name : " << dxl.first << ", ID : " << dxl.second
			   << ", Model Number : " << model_number);
    }

    return true;
}

bool
DynamixelController::initDynamixels()
{
    for (const auto& dxl : dynamixel_)
    {
	dxl_wb_->torqueOff((uint8_t)dxl.second);

	for (const auto& info : dynamixel_info_)
	{
	    if (const char* log;
		dxl.first == info.first &&
		info.second.item_name != "ID" &&
		info.second.item_name != "Baud_Rate" &&
		!dxl_wb_->itemWrite(dxl.second,
				    info.second.item_name.c_str(),
				    info.second.value, &log))
	    {
		RCLCPP_ERROR_STREAM(get_logger(),
				    log << ": Failed to write value["
				    << info.second.value << "] on items["
				    << info.second.item_name
				    << "] to Dynamixel[Name : "
				    << dxl.first << ", ID : "
				    << dxl.second << ']');
		return false;
	    }
	}

	dxl_wb_->torqueOn((uint8_t)dxl.second);
    }

    return true;
}

bool
DynamixelController::initControlItems()
{
    bool result = false;
    const char* log = NULL;

    auto it = dynamixel_.begin();

    const auto  goal_position = dxl_wb_->getItemInfo(it->second,
						     "Goal_Position");
    if (!goal_position)
	return false;
    control_items_["Goal_Position"] = goal_velocity;

    const auto	goal_velocity = dxl_wb_->getItemInfo(it->second,
						    "Goal_Velocity");
    if (!goal_velocity)
    {
	goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
	if (!goal_velocity)
	    return false;
    }
    control_items_["Goal_Velocity"] = goal_velocity;

    const auto	present_position = dxl_wb_->getItemInfo(it->second,
							"Present_Position");
    if (present_position == NULL)
	return false;
    control_items_["Present_Position"] = present_position;

    const auto	present_velocity = dxl_wb_->getItemInfo(it->second,
							"Present_Velocity");
    if (present_velocity == NULL)
    {
	present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
	if (present_velocity == NULL)
	    return false;
    }
    control_items_["Present_Velocity"] = present_velocity;

    const auto	present_current = dxl_wb_->getItemInfo(it->second,
						       "Present_Current");
    if (present_current == NULL)
    {
	present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
	if (present_current == NULL)
	    return false;
    }
    control_items_["Present_Current"] = present_current;

    return true;
}

bool
DynamixelController::initSDKHandlers()
{
    const char*	log = NULL;

    if (!dxl_wb_->addSyncWriteHandler(
	    control_items_["Goal_Position"]->address,
	    control_items_["Goal_Position"]->data_length, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
	return false;
    }
    else
	RCLCPP_INFO_STREAM(get_logger(), log);

    if (!dxl_wb_->addSyncWriteHandler(
	    control_items_["Goal_Velocity"]->address,
	    control_items_["Goal_Velocity"]->data_length, &log))
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
	return false;
    }
    else
    {
	RCLCPP_INFO_STREAM(get_logger(), log);
    }

    if (dxl_wb_->getProtocolVersion() == 2.0f)
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

	if (!dxl_wb_->addSyncReadHandler(start_address, read_length, &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(), log);
	    return false;
	}
    }

    return true;
}

bool
DynamixelController::getPresentPosition(const std::vector<std::string>& dxl_name)
{
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
	std::vector<uint8_t>	id_array;
	for (const auto& name : dxl_name)
	    id_array.push_back(dynamixel_[name]);

	std::vector<int32_t>	get_position(id_array.size());
	if (!dxl_wb_->syncRead(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(), &log)			||
	    !dxl_wb_->getSyncReadData(
		SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
		id_array.data(), id_array.size(),
		control_items_["Present_Position"]->address,
		control_items_["Present_Position"]->data_length,
		get_position.data(), &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(), log);
	    return false;
	}

	for(uint8_t index = 0; index < id_array.size(); ++index)
	{
	    WayPoint	wp;

	    wp.position	= dxl_wb_->convertValue2Radian(id_array[index],
						       get_position[index]);
	    wp.velocity	= 0.0f;
	    wp.acceleration = 0.0f;
	    pre_goal_.push_back(wp);
	}
    }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
    {
	for (auto const& dxl : dynamixel_)
	{
	    uint32_t read_position;

	    if (const char* log;
		!dxl_wb_->readRegister(
		    dxl.second,
		    control_items_["Present_Position"]->address,
		    control_items_["Present_Position"]->data_length,
		    &read_position, &log))
	    {
		RCLCPP_ERROR_STREAM(get_logger(), log);
		return false;
	    }

	    WayPoint	wp;
	    wp.position	    = dxl_wb_->convertValue2Radian(dxl.second,
							   read_position);
	    wp.velocity	    = 0.0f;
	    wp.acceleration = 0.0f;
	    pre_goal_.push_back(wp);
	}
    }

    return true;
}

void
DynamixelController::readCallback()
{
#ifdef DEBUG
    static double	priv_read_secs = get_clock()->now();
#endif
    bool result = false;
    const char* log = NULL;

    std::vector<dynamixel_state_t>	dynamixel_state(dynamixel_.size());
    dynamixel_state_list_.dynamixel_state.clear();

    int32_t get_current[dynamixel_.size()];
    int32_t get_velocity[dynamixel_.size()];
    int32_t get_position[dynamixel_.size()];

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    for (auto const& dxl:dynamixel_)
    {
	dynamixel_state[id_cnt].name = dxl.first;
	dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

	id_array[id_cnt++] = (uint8_t)dxl.second;
    }
#ifndef DEBUG
    if (is_moving_ == false)
    {
#endif
	if (dxl_wb_->getProtocolVersion() == 2.0f)
	{
	    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
				       id_array,
				       dynamixel_.size(),
				       &log);
	    if (result == false)
	    {
		ROS_ERROR("%s", log);
	    }

	    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
					      id_array,
					      id_cnt,
					      control_items_["Present_Current"]->address,
					      control_items_["Present_Current"]->data_length,
					      get_current,
					      &log);
	    if (result == false)
	    {
		ROS_ERROR("%s", log);
	    }

	    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
					      id_array,
					      id_cnt,
					      control_items_["Present_Velocity"]->address,
					      control_items_["Present_Velocity"]->data_length,
					      get_velocity,
					      &log);
	    if (result == false)
	    {
		ROS_ERROR("%s", log);
	    }

	    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
					      id_array,
					      id_cnt,
					      control_items_["Present_Position"]->address,
					      control_items_["Present_Position"]->data_length,
					      get_position,
					      &log);
	    if (result == false)
	    {
		ROS_ERROR("%s", log);
	    }

	    for(uint8_t index = 0; index < id_cnt; index++)
	    {
		dynamixel_state[index].present_current = get_current[index];
		dynamixel_state[index].present_velocity = get_velocity[index];
		dynamixel_state[index].present_position = get_position[index];

		dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
	    }
	}
	else if(dxl_wb_->getProtocolVersion() == 1.0f)
	{
	    uint16_t length_of_data = control_items_["Present_Position"]->data_length +
		control_items_["Present_Velocity"]->data_length +
		control_items_["Present_Current"]->data_length;
	    uint32_t get_all_data[length_of_data];
	    uint8_t dxl_cnt = 0;
	    for (auto const& dxl:dynamixel_)
	    {
		result = dxl_wb_->readRegister((uint8_t)dxl.second,
					       control_items_["Present_Position"]->address,
					       length_of_data,
					       get_all_data,
					       &log);
		if (result == false)
		{
		    ROS_ERROR("%s", log);
		}

		dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
		dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
		dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

		dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
		dxl_cnt++;
	    }
	}
#ifndef DEBUG
    }
#endif

#ifdef DEBUG
    ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
    priv_read_secs = ros::Time::now().toSec();
#endif
}

void
DynamixelController::publishCallback()
{
#ifdef DEBUG
    static double	priv_pub_secs = get_clock()->now();
#endif
    dynamixel_state_list_pub_.publish(dynamixel_state_list_);

    if (is_joint_state_topic_)
    {
	joint_state_msg_.header.stamp = get_clock()->now();

	joint_state_msg_.name.clear();
	joint_state_msg_.position.clear();
	joint_state_msg_.velocity.clear();
	joint_state_msg_.effort.clear();

	uint8_t id_cnt = 0;
	for (const auto& dxl : dynamixel_)
	{
	    const auto&	dxl_state = dynamixel_state_list_.dynamixel_state[id_cnt];
	    const auto	position = dxl_wb_->convertValue2Radian(
					dxl.second,
					int32_t(dxl_state.present_position));
	    const auto	velocity = dxl_wb_->convertValue2Velocity(
					dxl.second,
					int32_t(dxl_state.present_velocity));
	    const auto	effort	 = (dxl_wb_->getProtocolVersion() == 2.0f &&
				    strcmp(dxl_wb_->getModelName(dxl.second),
					   "XL-320") ?
				    dxl_wb_->convertValue2Load(
					int16_t(dxl_state.present_current)) :
				    dxl_wb_->convertValue2Current(
					int16_t(dxl_state.present_current)));

	    joint_state_msg_.name.push_back(dxl.first);
	    joint_state_msg_.position.push_back(position);
	    joint_state_msg_.velocity.push_back(velocity);
	    joint_state_msg_.effort.push_back(effort);

	    ++id_cnt;
	}

	joint_states_pub_.publish(joint_state_msg_);
    }

#ifdef DEBUG
    ROS_WARN("[publishCallback] diff_secs : %f",
	     (get_clock()->now() - priv_pub_secs).nanoseconds()*1.0e-9);
    priv_pub_secs = get_clock()->now();
#endif
}

void
DynamixelController::commandVelocityCallback(const twist_cp& twist)
{
    std::vector<uint8_t>	id_array;
    float			rpm = 0.0;
    for (auto const& dxl : dynamixel_)
    {
	const ModelInfo*	modelInfo = dxl_wb_->getModelInfo(dxl.second);
	rpm = modelInfo->rpm;
	id_array.push_back(dxl.second);
    }

  //  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
  //       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

    const uint8_t		LEFT  = 0;
    const uint8_t		RIGHT = 1;
    std::vector<double>		wheel_velocity[dynamixel_.size()];
    std::vector<int32_t>	dynamixel_velocity[dynamixel_.size()];

    double	velocity_constant_value = 1.0/(wheel_radius_ * rpm * 0.10472);


    wheel_velocity[LEFT]  = twist->linear.x
			  - (twist->angular.z * wheel_separation_ / 2);
    wheel_velocity[RIGHT] = twist->linear.x
			  + (twist->angular.z * wheel_separation_ / 2);

    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
	if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
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
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
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
	!dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
			    id_array.data(), id_array.size(),
			    dynamixel_velocity.data(), 1, &log);
    {
	RCLCPP_ERROR_STREAM(get_logger(), log);
    }
}

void
DynamixelController::writeCallback()
{
#ifdef DEBUG
    static auto	priv_pub_secs = get_clock()->now();
#endif
    if (is_moving_)
    {
	static uint32_t	point_cnt = 0;
	static uint32_t position_cnt = 0;


	std::vector<uint8_t>	id_array;
	for (const auto& joint : jnt_tra_msg_->joint_names)
	    id_array.push_back(dynamixel_[joint]);

	std::vector<uint32_t>	dynamixel_position;
	for (uint8_t index = 0; index < id_array.size(); ++index)
	    dynamixel_position.push_back(
		dxl_wb_->convertRadian2Value(
		    id_array[index],
		    jnt_tra_msg_->points[point_cnt].positions.at(index)));

	if (const char* log;
	    !dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
				id_array.data(), id_array.size(),
				dynamixel_position, 1, &log))
	{
	    RCLCPP_ERROR_STREAM(get_logger(), log);
	}

	++position_cnt;

	if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
	{
	    point_cnt++;
	    position_cnt = 0;
	    if (point_cnt >= jnt_tra_msg_->points.size())
	    {
		is_moving_ = false;
		point_cnt = 0;
		position_cnt = 0;

		RCLCPP_INFO_STREAM(get_logger(), "Complete Execution");
	    }
	}
    }

#ifdef DEBUG
    RCLCPP_WARN_STREAM(get_logger(),
		       "[writeCallback] diff_secs : "
		       << (get_clock()->now() - priv_pub_secs).nanoseconds()
		       * 1.0e-9);
    priv_pub_secs = get_clock()->now();
#endif
}

void
DynamixelController::trajectoryMsgCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{

    if (is_moving_)
    {
	RCLCPP_INFO_THROTTLE(get_logger(), 1, "Dynamixel is moving");
	return;
    }

    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    if (!getPresentPosition(msg->joint_names))
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Failed to get Present Position");
	return;
    }

    for (auto const& joint : msg->joint_names)
    {
	RCLCPP_INFO_STREAM(get_logger(),
			   "Joint '" << joint << "' is ready to move");

	jnt_tra_msg_->joint_names.push_back(joint);
    }

    if (jnt_tra_msg_->joint_names.size() == 0)
    {
	RCLCPP_WARN_STREAM(get_logger(), "Please check joint_name");
	return;
    }


    for (uint8_t cnt = 0; cnt < msg->points.size(); ++cnt)
    {
	std::vector<WayPoint> goal;
	for (size_t id_num = 0;
	     id_num < msg->points[cnt].positions.size(); ++id_num)
	{
	    WayPoint	wp;

	    wp.position = msg->points[cnt].positions.at(id_num);

	    if (msg->points[cnt].velocities.size())
		wp.velocity = msg->points[cnt].velocities.at(id_num);
	    else
		wp.velocity = 0.0f;

	    if (msg->points[cnt].accelerations.size())
		wp.acceleration = msg->points[cnt].accelerations.at(id_num);
	    else
		wp.acceleration = 0.0f;

	    goal.push_back(wp);
	}

	if (use_moveit_)
	{
	    trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

	    for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
	    {
		jnt_tra_point_msg.positions.push_back(goal[id_num].position);
		jnt_tra_point_msg.velocities.push_back(goal[id_num].velocity);
		jnt_tra_point_msg.accelerations.push_back(goal[id_num].acceleration);
	    }

	    jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
	}
	else
	{
	    jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

	    double move_time = 0.0f;
	    if (cnt == 0)
		move_time = msg->points[cnt].time_from_start.toSec();
	    else
		move_time = msg->points[cnt].time_from_start.toSec()
		    - msg->points[cnt-1].time_from_start.toSec();

	    jnt_tra_->init(move_time, write_period_, pre_goal_, goal);

	    std::vector<WayPoint> way_point;
	    trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

	    for (double index = 0.0;
		 index < move_time; index = index + write_period_)
	    {
		way_point = jnt_tra_->getJointWayPoint(index);

		for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
		{
		    jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
		    jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
		    jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
		}

		jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
		jnt_tra_point_msg.positions.clear();
		jnt_tra_point_msg.velocities.clear();
		jnt_tra_point_msg.accelerations.clear();
	    }

	    pre_goal_ = goal;
	}
    }

    RCLCPP_INFO_STREAM(get_logger(), "Succeeded to get joint trajectory!");
    is_moving_ = true;
}

void
DynamixelController::dynamixelCommandMsgCallback(
    const dynamixel_command_t::Request::SharedPtr req,
    dynamixel_command_t::Response::SharedPtr res)
{
    if (const char* log;
	!dxl_wb_->itemWrite(uint8_t(req->id), req->addr_name.c_str(),
			    int32_t(req->value), &log))
    {
	RCLCPP_ERROR_STREAM(get_logger,
			    log << "Failed to write value[" << value
			    << "] on items[" << item_name
			    << "] to Dynamixel[ID : " << id ']');
	res->comm_result = false;
    }
    else
	res->comm_result = true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_workbench_controllers");
    ros::NodeHandle node_handle("");

    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 57600;

    if (argc < 2)
    {
	ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
	return 0;
    }
    else
    {
	port_name = argv[1];
	baud_rate = atoi(argv[2]);
    }

    DynamixelController dynamixel_controller;

    bool result = false;

    std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

    result = dynamixel_controller.initWorkbench(port_name, baud_rate);
    if (result == false)
    {
	ROS_ERROR("Please check USB port name");
	return 0;
    }

    result = dynamixel_controller.getDynamixelsInfo(yaml_file);
    if (result == false)
    {
	ROS_ERROR("Please check YAML file");
	return 0;
    }

    result = dynamixel_controller.loadDynamixels();
    if (result == false)
    {
	ROS_ERROR("Please check Dynamixel ID or BaudRate");
	return 0;
    }

    result = dynamixel_controller.initDynamixels();
    if (result == false)
    {
	ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
	return 0;
    }

    result = dynamixel_controller.initControlItems();
    if (result == false)
    {
	ROS_ERROR("Please check control items");
	return 0;
    }

    result = dynamixel_controller.initSDKHandlers();
    if (result == false)
    {
	ROS_ERROR("Failed to set Dynamixel SDK Handler");
	return 0;
    }

    dynamixel_controller.initPublisher();
    dynamixel_controller.initSubscriber();
    dynamixel_controller.initServer();

    ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelController::readCallback, &dynamixel_controller);
    ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()), &DynamixelController::writeCallback, &dynamixel_controller);
    ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixel_controller);

    ros::spin();

    return 0;
}
}	// namespace dynamixel_workbench_controllers
