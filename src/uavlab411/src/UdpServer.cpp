#include "uavlab411/UdpServer.h"
/* ---- Global variable ---- */
// Socket server
int sockfd;
sockaddr_in android_addr;
socklen_t android_addr_size = sizeof(android_addr);
// waypoints vector
std::vector<uavlink_msg_waypoint_t> waypoint_indoor_vector;
std::vector<uavlink_msg_waypoint_t> waypoint_GPS_vector;
bool check_busy_fly_GPS_point;
bool check_take_off;
uint32_t lastest_id_waypoint = 0;
// uavpose
geometry_msgs::PoseStamped uavpose_msg;
ros::Duration _uavpose_timemout = ros::Duration(2.0);
// Timing
ros::Timer state_timeout_timer; // Check timeout connecting
ros::Duration arming_timeout;
ros::Duration state_timeout;

// ROS Service
ros::ServiceClient takeoff_srv, nav_to_waypoint_srv, land_srv, nav_to_GPS_srv;

// ROS Message
uavlab411::control_robot_msg msg_robot;		   // msg control robot
uavlab411::data_sensor_msg msg_sensor;         // msg sensor data
mavros_msgs::State state;					   // State robot
mavros_msgs::ManualControl manual_control_msg; // Manual control msg
sensor_msgs::NavSatFix global_msg;			   // message from topic "/mavros/global_position/global"
sensor_msgs::BatteryState battery_msg;		   // message from /mavros/battery

// param
int port;

// Service clients
ros::ServiceClient arming, set_mode;


// Publisher
ros::Publisher manual_control_pub;
ros::Publisher control_robot_pub;
bool check_receiver = false;
double yaw_compass;

void handle_cmd_set_mode(int mode)
{
	if (mode > end(mode_define) - begin(mode_define) - 1)
	{
		ROS_ERROR("Error calling set_mode, mode index invalid: %d", mode);
	}
	else
	{
		if (state.mode != mode_define[mode])
		{
			static mavros_msgs::SetMode sm;
			sm.request.custom_mode = mode_define[mode];

			if (!set_mode.call(sm))
			{
				ROS_INFO("Error calling set_mode service");
				throw std::runtime_error("Error calling set_mode service");
			}
		}
		else
		{
			ROS_INFO("UAV already in this mode");
		}
	}
}

void handle_cmd_arm_disarm(bool flag)
{
	if (!TIMEOUT(state, state_timeout) && !state.armed && flag == true) // Arming
	{
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!arming.call(srv))
		{
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok())
		{
			if (state.armed)
			{
				break;
			}
			else if (ros::Time::now() - start > arming_timeout)
			{
				string report = "Arming timed out";
				ROS_INFO("ARMING TIMEOUT... TRY AGAIN!!");
				break;
			}
		}
	}
	else if (!TIMEOUT(state, state_timeout) && state.armed && flag == false) // Disarming
	{
		ROS_INFO("DISARM"); // TODO: handle disarm motor
	}
}

void handle_cmd_takeoff(float altitude)
{
	uavlab411::Takeoff takeoff;
	takeoff.request.z = altitude;

	if (takeoff_srv.call(takeoff))
	{
		ROS_INFO("CALLED TAKEOFF SRV!");
		check_take_off = true;
	}
	else
	{
		ROS_ERROR("Failed to call service takeoff");
		return;
	}
	return;
}

void handle_cmd_land()
{
	std_srvs::Trigger land;
	if (land_srv.call(land))
	{
		msg_robot.step2 = 100;
		msg_robot.step3 = 136;
		msg_robot.step4 = 160;
		control_robot_pub.publish(msg_robot);
		ROS_INFO("CALLED LAND SRV!");
		check_take_off = false;
		check_busy_fly_GPS_point = false;
	}
	else
	{
		ROS_ERROR("Failed to call service land");
		return;
	}
	return;
}

void handle_cmd_flyto(bool allwp, int wpid, int type)
{
	if (!check_take_off)
		ROS_ERROR("Error calling service navigate waypoints, take off first!");
	else
	{
		int type_fly = type;
		ROS_INFO("WP INDOOR LENGTH: %ld", waypoint_indoor_vector.size());
		ROS_INFO("WP OUTDOOR LENGTH: %ld", waypoint_GPS_vector.size());
		if (!check_busy_fly_GPS_point)
		{
			ROS_INFO("Start flying to waypoints in mode %s", type == 0 ? "indoor" : "outdoor");
			std::thread flyThread(&navigate_points_vector, &type_fly);
			flyThread.detach();
		}
		else
			ROS_ERROR("Error: uav has been flying to waypoint!");
	}
}

void handle_command(uavlink_message_t message)
{
	uavlink_command_t command_msg;
	uavlink_command_decode(&message, &command_msg);
	ROS_INFO("cmd: %d", command_msg.command);
	switch (command_msg.command)
	{
	case UAVLINK_CMD_SET_MODE:
		handle_cmd_set_mode((int)command_msg.param1);
		break;

	case UAVLINK_CMD_ARM_DISARM:
		handle_cmd_arm_disarm((bool)command_msg.param1);
		break;

	case UAVLINK_CMD_TAKEOFF:
		handle_cmd_takeoff((float)command_msg.param1);
		break;
	case UAVLINK_CMD_FLYTO:
		handle_cmd_flyto((bool)command_msg.param1, (int)command_msg.param2, (int)command_msg.param3);
		break;
	case UAVLINK_CMD_LAND:
		handle_cmd_land();
		break;
	default:
		break;
	}
}
void handle_msg_control_robot(uavlink_message_t message)
{
	uavlink_control_robot_t robot_msg_rev;
	uavlink_control_robot_decode(&message, &robot_msg_rev);
	msg_robot.step1 = robot_msg_rev.step1;
	msg_robot.step2 = robot_msg_rev.step2;
	msg_robot.step3 = robot_msg_rev.step3;
	msg_robot.step4 = robot_msg_rev.step4;
	msg_robot.step5 = robot_msg_rev.step5;
	control_robot_pub.publish(msg_robot);
}
void handle_msg_manual_control(uavlink_message_t message)
{
	uavlink_msg_manual_control manual_msg;
	uavlink_manual_control_decode(&message, &manual_msg);
	manual_control_msg.x = manual_msg.x;
	manual_control_msg.y = manual_msg.y;
	manual_control_msg.z = manual_msg.z;
	manual_control_msg.r = manual_msg.r;

	manual_control_pub.publish(manual_control_msg);
}
// Handle waupoint message
void handle_msg_waypoint(uavlink_message_t message)
{
	
	uavlink_msg_waypoint_t waypoint;
	uavlink_waypoint_decode(&message, &waypoint);
	// ROS_INFO("msg rev:x= %f,y=%f,z=%f",waypoint.targetX,waypoint.targetY,waypoint.targetZ);
	if (waypoint.type == 0) waypoint_indoor_vector.push_back(waypoint);
	else if (waypoint.type == 1)
	{
		if (waypoint.wpId > lastest_id_waypoint)
		{
			waypoint_GPS_vector.push_back(waypoint);
			lastest_id_waypoint = waypoint.wpId;
		}
	}
		
	else
		ROS_ERROR("Error: type of message waypoint received is invalid ! : type is %d", waypoint.type);
}
// Handle state from UAV
void handleState(const mavros_msgs::State &s)
{
	state = s;
	uavlink_state_t send_state;
	send_state.armed = s.armed;
	send_state.connected = s.connected;
	send_state.mode = mode_to_int(s.mode);
	send_state.battery_remaining = battery_remaining_calculate(battery_msg.voltage);

	uavlink_message_t msg;
	uavlink_state_encode(&msg, &send_state);

	char buf[300];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
	writeSocketMessage(buf, len);
}

// Handle Local Position from UAV
void handleLocalPosition(const nav_msgs::Odometry &o)
{
	ros::Rate r(10);
	uavlink_global_position_int_t global_pos;
	global_pos.vx = (int16_t)(o.twist.twist.linear.x * 100);
	global_pos.vy = (int16_t)(o.twist.twist.linear.y * 100);
	global_pos.vz = (int16_t)(o.twist.twist.linear.z * 100);

	// get data from global_position
	global_pos.alt = (int16_t)(o.pose.pose.position.z * 100);
	global_pos.lat = (int32_t)(global_msg.latitude * 10000000);
	global_pos.lon = (int32_t)(global_msg.longitude * 10000000);
	global_pos.yaw = (int16_t)(yaw_compass * 180 / PI);
	global_pos.yaw = 450 - global_pos.yaw > 360 ? 90 - global_pos.yaw : 450 - global_pos.yaw;
	uavlink_message_t msg;
	uavlink_global_position_encode(&msg, &global_pos);
	char buf[300];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
	writeSocketMessage(buf, len);
	r.sleep();
}

// Handle global Posotion from UAV
void handleGlobalPosition(const sensor_msgs::NavSatFix &n)
{
	global_msg = n;
}
// Handle sensor data from node sensor
void handleSensorData(const uavlab411::data_sensor_msg &data_from_node)
{
	
	uavlink_msg_sensor_data_t data;
	char buf[300];
	data.id = data_from_node.id;
	data.lat = (int32_t)(data_from_node.lat*10000000);
	data.lon = (int32_t)(data_from_node.lon*10000000);
	data.temperature = (int16_t)(data_from_node.temp*100);
	data.humidity = (int16_t)(data_from_node.hum*100);
	data.gas = (int16_t)(data_from_node.gas*100);
	uavlink_message_t msg_sensor;
	uavlink_sensor_data_encode(&msg_sensor, &data);
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg_sensor);
	writeSocketMessage(buf, len);
	ROS_INFO("id sensor: %d",data.id);
	
}
void handleUavPose(const geometry_msgs::PoseStampedConstPtr &_uavpose)
{
	uavlink_local_position_int_t uavpose;
	uavpose_msg.pose = _uavpose->pose;
	uavpose_msg.header = _uavpose->header;
	uavpose.posX = (int16_t)(_uavpose->pose.position.x * 1000);
	uavpose.posY = (int16_t)(_uavpose->pose.position.y * 1000);
	uavpose.posZ = (int16_t)(_uavpose->pose.position.z * 1000);
	uavpose.vx = 0;
	uavpose.vy = 0;
	uavpose.vz = 0;
	uavlink_message_t msg;
	uavlink_local_position_encode(&msg, &uavpose);
	char buf[100];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
	writeSocketMessage(buf, len);
}

// Handle battery state from UAV
void handle_Battery_State(const sensor_msgs::BatteryState &bat)
{
	battery_msg = bat;
}

void handleImuData(const sensor_msgs::Imu::ConstPtr &msg)
{
	tf::Quaternion q(
		msg->orientation.x,
		msg->orientation.y,
		msg->orientation.z,
		msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw_compass);
}

void init()
{
	// Thread for UDP soket read
	std::thread readThread(&readingSocketThread);
	readThread.detach();
}

bool navigate_to_local(uavlink_msg_waypoint_t point, float tolerance)
{
	uavlab411::Navigate navigate;
	navigate.request.x = point.targetX;
	navigate.request.y = point.targetY;
	navigate.request.z = point.targetZ;
	navigate.request.speed = 0;
	navigate.request.nav_mode = 3;
	navigate.request.tolerance = tolerance;

	if (nav_to_waypoint_srv.call(navigate))
		ROS_INFO("CALLED NAV SRV!");
	else
	{
		ROS_ERROR("Failed to call service nav");
		return false;
	}
	ros::Time start = ros::Time::now();
	while (true)
	{
		if (TIMEOUT(uavpose_msg, _uavpose_timemout))
		{
			ROS_INFO("nav to waypoint err: time out uavpose");
			return false;
		}
		if (point.targetX - uavpose_msg.pose.position.x < tolerance && point.targetY - uavpose_msg.pose.position.y < tolerance && point.targetZ - uavpose_msg.pose.position.z < tolerance)
		{
			ROS_INFO("nav to waypoint x:%f,y:%f,z:%f success", point.targetX, point.targetY, point.targetZ);
			return true;
		}
		// if (ros::Time::now() - start > ros::Duration(10))
		// {
		// 	ROS_INFO("nav to waypoint err: over 10s -> fly to next waypoint");
		// 	return true;
		// }
		ros::Duration(0.2).sleep();
	}
}
bool navigate_to_GPS(uavlink_msg_waypoint_t point, float tolerance)
{
	uavlab411::NavigateGlobal msg;
	msg.request.lat = point.targetX;
	msg.request.lon = point.targetY;
	msg.request.alt = point.targetZ;
	msg.request.speed = 0.8;
	msg.request.tolerance = tolerance;

	if (nav_to_GPS_srv.call(msg))
		ROS_INFO("CALLED SERVICE NAVIGATE GPS!");
	else
		ROS_ERROR("Failed to call service nav GPS");

	ros::Time start = ros::Time::now();
	while (true)
	{
		float distance;
		distance = get_distance_GPS(double(global_msg.latitude), double(global_msg.longitude), point.targetX, point.targetY);

		if (distance < tolerance)
		{
			ROS_INFO("nav to waypoint lat:%f,lon:%f,z:%f success", point.targetX, point.targetY, point.targetZ);
			return true;
		}

		// if (ros::Time::now() - start > ros::Duration(10))
		// {
		// 	ROS_ERROR("nav to waypoint err: over 10s -> fly to next waypoint");
		// 	return true;
		// }
		ros::Duration(0.2).sleep();
	}
}

void navigate_points_vector(void *type)
{
	int type_fly = *(int *)type;
	check_busy_fly_GPS_point = true;
	ROS_INFO("Type fly: %d", type_fly);
	if (type_fly == 0)
	{
		while (!waypoint_indoor_vector.empty())
		{
			ROS_INFO("fly to point x: %f y:%f z:%f", waypoint_indoor_vector[0].targetX, waypoint_indoor_vector[0].targetY, waypoint_indoor_vector[0].targetZ);
			if (navigate_to_local(waypoint_indoor_vector[0], 0.1))
			{
				waypoint_indoor_vector.erase(waypoint_indoor_vector.begin());
			}
		}
	}
	else if (type_fly == 1)
	{
		while (!waypoint_GPS_vector.empty())
		{
			ROS_INFO("fly to point lat: %f lon:%f z:%f", waypoint_GPS_vector[0].targetX, waypoint_GPS_vector[0].targetY, waypoint_GPS_vector[0].targetZ);
			if (navigate_to_GPS(waypoint_GPS_vector[0], 0.3))
			{
				waypoint_GPS_vector.erase(waypoint_GPS_vector.begin());
			}
		}
	}
	else
	{
		ROS_ERROR("Error: invalid type fly : %d", type_fly);
	}

	check_busy_fly_GPS_point = false;
}

int createSocket(int port)
{
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(port);

	if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0)
	{
		ROS_FATAL("socket bind error: %s", strerror(errno));
		close(sockfd);
		ros::shutdown();
	}

	return sockfd;
}

void readingSocketThread()
{
	char buff[1024];

	// Socket create
	sockfd = createSocket(port);
	memset(&android_addr, 0, sizeof(android_addr));
	ROS_INFO("UDP UdpSocket initialized on port %d", port);

	while (true)
	{
		// read next UDP packet
		int bsize = recvfrom(sockfd, (char *)buff, 1024, 0, (sockaddr *)&android_addr, &android_addr_size);

		buff[bsize] = '\0';
		if (bsize < 0)
		{
			ROS_ERROR("recvfrom() error: %s", strerror(errno));
		}
		else
		{
			if (!check_receiver)
				check_receiver = true;
			uavlink_message_t message;
			memcpy(&message, buff, bsize);
			
			switch (message.msgid)
			{
			case UAVLINK_MSG_ID_MANUAL_CONTROL:
				handle_msg_manual_control(message);
				break;

			case UAVLINK_MSG_ID_COMMAND:
				handle_command(message);
				break;

			case UAVLINK_CONTROL_ROBOT_MSG_ID:
				handle_msg_control_robot(message);
				break;
			case UAVLINK_MSG_ID_WAYPOINT:
				handle_msg_waypoint(message);
				break;
			default:
				break;
			}
		}
	}
}

void writeSocketMessage(char buff[], int length)
{
	if (check_receiver) // Need received first
	{
		int len = sendto(sockfd, (const char *)buff, length, 0, (const struct sockaddr *)&android_addr, android_addr_size);
		// ROS_INFO("length: %d",len);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UdpSocket");
	ros::NodeHandle nh, nh_priv("~");

	// param
	nh_priv.param("port", port, 12345);

	// Initial publisher
	manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
	control_robot_pub = nh.advertise<uavlab411::control_robot_msg>("uavlab411/control_robot", 1);

	// Initial subscribe
	auto state_sub = nh.subscribe("mavros/state", 1, &handleState);
	auto global_position_sub = nh.subscribe("/mavros/global_position/global", 1, &handleGlobalPosition);
	auto local_position_sub = nh.subscribe("/mavros/global_position/local", 1, &handleLocalPosition);
	auto battery_sub = nh.subscribe("/mavros/battery", 1, &handle_Battery_State);
	auto uavpose_sub = nh.subscribe("uavlab411/uavpose", 1, &handleUavPose);
	auto data_sensor_sub = nh.subscribe("/uavlab411/sensor_data",1,&handleSensorData);
	auto sub_imu_data = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &handleImuData);

	// Service client
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_srv = nh.serviceClient<uavlab411::Takeoff>("uavlab411/takeoff");
	nav_to_waypoint_srv = nh.serviceClient<uavlab411::Navigate>("uavlab411/navigate");
	nav_to_GPS_srv = nh.serviceClient<uavlab411::NavigateGlobal>("uavlab411/navigate_global");
	land_srv = nh.serviceClient<std_srvs::Trigger>("uavlab411/land");

	// Timer
	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));

	init();
	ros::spin();
}
