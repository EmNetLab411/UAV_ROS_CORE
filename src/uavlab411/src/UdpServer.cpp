#include "uavlab411/UdpServer.h"

// Ensure circle command macro exists (if header not in sync)
#ifndef UAVLINK_CMD_CIRCLE
#define UAVLINK_CMD_CIRCLE 28
#endif

// Forward declaration in case header wasn't included before switch usage
void handle_cmd_circle(bool start);

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>

// Robotic Arm library
#include <pca9685_servo_control/SetAngle.h>

/* ---- Global variable ---- */
// Socket server
int sockfd;
sockaddr_in android_addr;
socklen_t android_addr_size = sizeof(android_addr);
// waypoints vector
std::vector<uavlink_msg_waypoint_t> waypoint_indoor_vector;
std::vector<uavlink_msg_waypoint_t> waypoint_GPS_vector;
bool check_busy;
bool check_take_off;

// circle control
std::atomic<bool> circle_active(false);
std::thread circle_thread;

// uavpose
geometry_msgs::PoseStamped uavpose_msg;
ros::Duration _uavpose_timemout = ros::Duration(2.0);
// Timing
ros::Timer state_timeout_timer; // Check timeout connecting
ros::Duration arming_timeout;
ros::Duration state_timeout;

// ROS Service
ros::ServiceClient takeoff_srv, nav_to_waypoint_srv, land_srv, nav_to_GPS_srv;

// ROBOTIC ARM
ros::ServiceClient set_angle_cli_;  // client gọi /pca9685_servo/set_angle
void initServoBridge(ros::NodeHandle& nh);

// Training data record 
ros::ServiceClient dataset_toggle_srv; // for /dataset_logger/toggle

// ROS Message
uavlab411::control_robot_msg msg_robot;        // msg control robot
mavros_msgs::State state;					   // State robot
mavros_msgs::ManualControl manual_control_msg; // Manual control msg
sensor_msgs::NavSatFix global_msg;			   // message from topic "/mavros/global_position/global"
sensor_msgs::BatteryState battery_msg;		   // message from /mavros/battery

mavros_msgs::OverrideRCIn rc_msg; // RC message

// param
int port;

// Service clients
ros::ServiceClient arming, set_mode;

// Subcriber
ros::Subscriber state_sub;

// Publisher
ros::Publisher manual_control_pub;
ros::Publisher control_robot_pub;
ros::Publisher rc_override_pub; // add publisher RC override
bool check_receiver = false;

// ROS Publisher for position in offboard
ros::Publisher position_control_pub;
geometry_msgs::PoseStamped position_cmd_msg;

// velocity control publishers offboard mode - get data for AI training
ros::Publisher vel_cmd_pub_stamped;    // /mavros/setpoint_velocity/cmd_vel (TwistStamped)
ros::Publisher vel_cmd_pub_unstamped;  // /mavros/setpoint_velocity/cmd_vel_unstamped (Twist)

// Position control state
bool position_control_active = false;
ros::Time last_position_cmd_time;
ros::Duration position_cmd_timeout = ros::Duration(2.0);

// Drone status
double local_x = 0.0;   // local x
double local_y = 0.0;   // local y
double local_z = 0.0;   // altitude

geometry_msgs::TwistStamped velocity_msg; // velocity
sensor_msgs::Imu imu_msg; // orientation

// Setup Vx override for training
static bool   vx_override_enable = false;
static double vx_override_value  = 0.05; // m/s

// Altitude hold (minimal)
static bool   alt_hold_enable = true;
static double alt_target_z    = 0.5;  // m (ENU up)
static double kp_z            = 1.0;
static double kd_z            = 0.6;
static double max_z_vel       = 0.5;  // m/s
static bool   invert_z_sign   = false;

static inline double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// PD giữ độ cao: dùng local_z (đã cập nhật ở handleLocalPose) và vz đo từ /mavros/local_position/velocity_local
static inline double compute_vz_hold() {
    const double z_now   = local_z;                       // m
    const double vz_meas = velocity_msg.twist.linear.z;   // m/s (ENU)
    const double err  = alt_target_z - z_now;             // ENU up
    double vz_cmd = kp_z * err - kd_z * vz_meas;          // PD
    if (invert_z_sign) vz_cmd = -vz_cmd;
    return clampd(vz_cmd, -max_z_vel, max_z_vel);
}

// --------------------------------ROBOTIC ARM------------------------------- //
void UdpServer::initServoBridge(ros::NodeHandle& nh) {
  // Chuẩn bị ROS service client cho servo
  if (!set_angle_cli_) {
    set_angle_cli_ = nh.serviceClient<pca9685_servo_control::SetAngle>("/pca9685_servo/set_angle");
  }
  ROS_INFO("[UdpServer] Servo bridge ready: service [/pca9685_servo/set_angle]");
}

void handle_msg_servo_control(const uavlink_message_t& msg) {
  // Giả định payload do client gửi: [uint8 channel][float32 angle_deg] (little-endian)
  // CHỌN 1 TRONG 2 CÁCH DECODE DƯỚI (tùy bạn đã có generator decode hay chưa):

  // CÁCH A (nếu bạn đã có mã tạo sẵn giống các message khác):
  // struct uavlink_servo_control_t { uint8_t channel; float angle_deg; };
  // uavlink_servo_control_t pkt{};
  // uavlink_msg_servo_control_decode(&msg, &pkt);
  // uint8_t channel = pkt.channel;
  // float angle_deg = pkt.angle_deg;

  // CÁCH B (fallback: decode thủ công theo định dạng nêu trên)
  uint8_t channel = 0;
  float angle_deg = 0.0f;
  // Lưu ý: thay 'msg.payload' và 'msg.len' theo tên trường thực tế trong uavlink_message_t của bạn.
  // Dưới đây là ví dụ thông dụng; chỉnh lại nếu khác.

  const uint8_t* p = reinterpret_cast<const uint8_t*>(msg.payload);
  if (msg.len >= (int)(sizeof(uint8_t) + sizeof(float))) {
    channel = p[0];
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::memcpy(&angle_deg, p + 1, sizeof(float));
  } else {
    ROS_WARN("SERVO_CONTROL payload too short: len=%d", msg.len);
    return;
  }

  // Gọi ROS service /pca9685_servo/set_angle
  if (!set_angle_cli_.exists()) {
    set_angle_cli_.waitForExistence(ros::Duration(0.5));
  }

  pca9685_servo_control::SetAngle srv;
  srv.request.channel   = channel;
  srv.request.angle_deg = angle_deg;

  if (set_angle_cli_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Servo ch=%u -> %.1f deg OK: %s", channel, angle_deg, srv.response.message.c_str());
    } else {
      ROS_WARN("Servo ch=%u -> %.1f deg FAILED: %s", channel, angle_deg, srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Failed calling /pca9685_servo/set_angle (ch=%u, deg=%.1f)", channel, angle_deg);
  }
}

// ----------------------------------------------------------------------------- //

void handle_cmd_set_mode(int mode)
{
    if (mode >= 0 && mode < (int)(sizeof(mode_define)/sizeof(mode_define[0])))
    {
        if (state.mode != mode_define[mode])
        {
            mavros_msgs::SetMode sm;
            sm.request.custom_mode = mode_define[mode];

            if (!set_mode.call(sm))
            {
                ROS_ERROR("Error calling set_mode service");
            }
            else
            {
                ROS_INFO("Set mode to: %s", mode_define[mode].c_str());
            }
        }
    }
    else
    {
        ROS_ERROR("Invalid mode index: %d", mode);
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
	ROS_INFO("[ALTITUDE] alt=%.2f", altitude);

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
		ROS_INFO("CALLED LAND SRV!");
		check_take_off = false;
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
		if (!check_busy)
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
	// debug: show raw payload bytes and length
    ROS_DEBUG("handle_command: msgid=%u len=%u", message.msgid, message.len);
    std::string hex;
    for (int i = 0; i < message.len; ++i)
    {
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%02X ", (unsigned char)message.payload64[i]);
        hex += tmp;
    }
    ROS_DEBUG("command payload: %s", hex.c_str());


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
			//position control in offboard
		case UAVLINK_CMD_POSITION_CONTROL_MODE:
			handle_cmd_position_control_mode((bool)command_msg.param1);
			break;

		case UAVLINK_CMD_CIRCLE:
			handle_cmd_circle((bool)command_msg.param1); // param1=true to start, false to stop
			break;

		case UAVLINK_CMD_TRAIN_TOGGLE: {
			bool enable = (bool)command_msg.param1;     // 1=bật, 0=tắt
			if (!dataset_toggle_srv.exists()) {
				dataset_toggle_srv.waitForExistence(ros::Duration(2.0));
			}
			std_srvs::SetBool srv; srv.request.data = enable;
			if (dataset_toggle_srv.call(srv)) {
				ROS_INFO("[UdpServer] Training logging %s (%s)",
						enable ? "ENABLED" : "DISABLED",
						srv.response.message.c_str());
			} else {
				ROS_ERROR("[UdpServer] Failed to call /dataset_logger/toggle");
			}
			break;
		}

		case UAVLINK_CMD_VX_OVERRIDE: {
			bool en = static_cast<bool>(command_msg.param1);  // 1=bật, 0=tắt
			vx_override_enable = en;
			ROS_INFO("[UdpServer] vx override %s (default vx=%.3f m/s)",
					en ? "ENABLED" : "DISABLED", vx_override_value);
			break;
		}

		default:
			break;
	}
}

// Helper: lấy yaw hiện tại (rad) từ IMU; fallback = 0 nếu quaternion không hợp lệ - #AI
static inline double get_current_yaw()
{
    const auto& o = imu_msg.orientation;
    double qx = o.x, qy = o.y, qz = o.z, qw = o.w;
    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 0.9 || norm > 1.1 ||
        std::isnan(qx) || std::isnan(qy) || std::isnan(qz) || std::isnan(qw))
    {
        return 0.0;
    }
    tf::Quaternion q(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw; // ENU yaw
}


void handle_msg_control_robot(uavlink_message_t message)
{
	uavlink_control_robot_t robot_msg_rev;
	uavlink_control_robot_decode(&message,&robot_msg_rev);
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

	//Convert to fit MAVROS manual control
	manual_control_msg.x = manual_msg.x;
	manual_control_msg.y = manual_msg.y;
	manual_control_msg.z = manual_msg.z;
	manual_control_msg.r = manual_msg.r;

	manual_control_pub.publish(manual_control_msg);
}

/* ******************Offboard MODE********************** */


// Function position control message
void handle_msg_position_control(uavlink_message_t message)
{
    uavlink_position_control_t position_msg;
    uavlink_position_control_decode(&message, &position_msg);

	//Update Timer
	last_position_cmd_time = ros::Time::now();
    
    // ROS_INFO("Position control: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, frame=%d",
    //          position_msg.x, position_msg.y, position_msg.z, position_msg.yaw, position_msg.frame);
    
    if (!position_control_active) {
        // ROS_WARN("Position control not active. Enable position control mode first.");
        send_position_feedback(false, 0, 0, 0);
        return;
    }
    
    // Prepare message for ROS
    position_cmd_msg.header.stamp = ros::Time::now();
    position_cmd_msg.header.frame_id = "map";
    
    if (position_msg.frame == 0) { // Local frame
        position_cmd_msg.pose.position.x = position_msg.x;
        position_cmd_msg.pose.position.y = position_msg.y;
        position_cmd_msg.pose.position.z = position_msg.z;
    } else {
        // Global frame (GPS)
        // For Clover drone, we'll use local frame primarily
        ROS_WARN("Global frame position control not implemented. Using local frame.");
        position_cmd_msg.pose.position.x = position_msg.x;
        position_cmd_msg.pose.position.y = position_msg.y;
        position_cmd_msg.pose.position.z = position_msg.z;
    }

	double yaw_cmd = std::isfinite(position_msg.yaw) ? position_msg.yaw : get_current_yaw();
    
	// Use tf2 to create quaternion from yaw (keep roll/pitch = 0)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_cmd); // roll, pitch, yaw
    position_cmd_msg.pose.orientation = tf2::toMsg(q);
    
    // Publish message
    position_control_pub.publish(position_cmd_msg);
    last_position_cmd_time = ros::Time::now();
    
    // Gửi feedback thành công
    send_position_feedback(true, 0, 0, 0);
}
// Function on/off position control Offboard
void handle_cmd_position_control_mode(bool enable)
{
    if (enable) {
        // Set mode to OFFBOARD để nhận position commands
        handle_cmd_set_mode(3); // 3 = OFFBOARD
        position_control_active = true;
        ROS_INFO("Offboard control mode enabled");
    } else {
        // Return manual or posctl
        // handle_cmd_set_mode(0);
        position_control_active = false;
        ROS_INFO("Offboard control mode disabled");
    }
}
// Function feedback to position
void send_position_feedback(bool success, float error_x, float error_y, float error_z)
{
    uavlink_position_feedback_t feedback;
    feedback.success = success ? 1 : 0;
    feedback.error_x = error_x;
    feedback.error_y = error_y;
    feedback.error_z = error_z;
    
    uavlink_message_t msg;
    uavlink_position_feedback_encode(&msg, &feedback);
    
    char buf[300];
    uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
    writeSocketMessage(buf, len);
}

// Function velocity control message - #AI training
void handle_msg_velocity_control(uavlink_message_t message)
{
    uavlink_velocity_control_t vc;
    uavlink_velocity_control_decode(&message, &vc);

	ROS_INFO("[VELOCITY] vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f frame=%d",
             vc.vx, vc.vy, vc.vz, vc.yaw_rate, vc.frame);

    geometry_msgs::TwistStamped ts;
    ts.header.stamp = ros::Time::now();

    // Chuẩn Clover: ENU local frame ("map")
    // Nếu client gửi body frame (frame==1), transform body -> ENU bằng yaw hiện tại
    double vx_enu = vc.vx;
    double vy_enu = vc.vy;
    double vz_enu = vc.vz;

    if (vc.frame == 1) {
        double yaw = get_current_yaw();
        double c = std::cos(yaw), s = std::sin(yaw);
        // Body(X,Y) -> ENU(X,Y)
        double x_e = c * vc.vx - s * vc.vy;
        double y_e = s * vc.vx + c * vc.vy;
        vx_enu = x_e;
        vy_enu = y_e;
        // z giữ nguyên (body Z song song ENU Z khi roll/pitch nhỏ)
    }

    ts.header.frame_id = "map"; // publish trong ENU local frame

	// Apply Vx override if enabled (for training)
	if (vx_override_enable) 
	{
		vx_enu = vx_override_value;
	}
    ts.twist.linear.x  = vx_enu;
    ts.twist.linear.y  = vy_enu;

	// giữ độ cao nếu alt_hold_enable=true
    // ts.twist.linear.z  = vz_enu;
	ts.twist.linear.z  = alt_hold_enable ? compute_vz_hold() : vz_enu;

    ts.twist.angular.x = 0.0;
    ts.twist.angular.y = 0.0;
    ts.twist.angular.z = vc.yaw_rate; // yaw rate (rad/s)

    if (vel_cmd_pub_stamped) vel_cmd_pub_stamped.publish(ts);

    geometry_msgs::Twist t = ts.twist;
    if (vel_cmd_pub_unstamped) vel_cmd_pub_unstamped.publish(t);
}

/********************** Mission Function *******************************/
// internal runner: radius (m), altitude (m), linear speed (m/s)
static void circle_runner(double radius, double altitude, double speed)
{
    // ensure Offboard mode
    // handle_cmd_set_mode(3); // request OFFBOARD

    // wait small time for offboard to activate
    ros::Time start_wait = ros::Time::now();
    ros::Duration wait_timeout = ros::Duration(3.0);
    while (ros::ok() && ros::Time::now() - start_wait < wait_timeout)
    {
        if (state.mode == std::string("OFFBOARD")) break;
        ros::Duration(0.05).sleep();
    }

	double cx = local_x;
	double cy = local_y;
	
    if (std::isnan(cx) || std::isnan(cy))
    {
        cx = 0.0; cy = 0.0;
    }

    double ang = 0.0;
    // angular speed (rad/s) from linear speed v = r * omega -> omega = v / r
    double omega = (radius > 1e-6) ? (speed / radius) : 0.5;

    ros::Rate rate(20.0); // publish setpoints at 20 Hz
    geometry_msgs::PoseStamped cmd;
    cmd.header.frame_id = "map";

    while (ros::ok() && circle_active.load())
    {
        ang += omega * (1.0 / 20.0); // dt = 1/20
        double x = cx + radius * cos(ang);
        double y = cy + radius * sin(ang);
        double z = altitude;

        // set position
        cmd.header.stamp = ros::Time::now();
        cmd.pose.position.x = x;
        cmd.pose.position.y = y;
        cmd.pose.position.z = z;

        // yaw tangent direction (face along velocity)
        double yaw = ang + M_PI_2;
        double cyaw = cos(yaw * 0.5);
        double syaw = sin(yaw * 0.5);
        // roll/pitch zero
        cmd.pose.orientation.w = cyaw;
        cmd.pose.orientation.x = 0.0;
        cmd.pose.orientation.y = 0.0;
        cmd.pose.orientation.z = syaw;

        position_control_pub.publish(cmd);

        rate.sleep();
    }

    // when stopping, optionally publish hold position at center
    if (ros::ok())
    {
        geometry_msgs::PoseStamped hold = cmd;
        hold.pose.position.x = cx;
        hold.pose.position.y = cy;
        hold.pose.position.z = altitude;
        position_control_pub.publish(hold);
    }
}

// handler called from command parser
void handle_cmd_circle(bool start)
{
    if (start)
    {
        if (circle_active.load())
        {
            ROS_WARN("Circle already active");
            return;
        }

        // radius fixed 0.5m, altitude from current local pose or fallback 1.5, speed 0.5 m/s
        double radius = 0.5;
        double altitude = uavpose_msg.pose.position.z;
        if (std::isnan(altitude) || altitude < 0.1) altitude = 1.5;
        double speed = 0.5;

        circle_active.store(true);
        circle_thread = std::thread(circle_runner, radius, altitude, speed);
        circle_thread.detach();

        ROS_INFO("Started circle: radius=%.2fm alt=%.2fm speed=%.2fm/s", radius, altitude, speed);
    }
    else
    {
        if (!circle_active.load())
        {
            ROS_WARN("Circle not active");
            return;
        }
        circle_active.store(false);
        ROS_INFO("Stopping circle");
        // thread will exit on next loop iteration
    }
}

/* ************************* Function Servo Control ***********************************/
static inline void uavlink_servo_channels_decode(const uavlink_message_t *msg, uavlink_servo_channels_t *sc)
{
    if (!msg || !sc) return;
    uint8_t len = msg->len < UAVLINK_MSG_ID_SERVO_CHANNELS_LEN ? msg->len : UAVLINK_MSG_ID_SERVO_CHANNELS_LEN;
    memset(sc, 0, UAVLINK_MSG_ID_SERVO_CHANNELS_LEN);
    memcpy(sc, _MAV_PAYLOAD(msg), len);
}

void handle_msg_servo_channels(uavlink_message_t message)
{
    uavlink_servo_channels_t sc;
    uavlink_servo_channels_decode(&message, &sc);

    // Đảm bảo service client sẵn sàng (đã init ở initServoBridge)
    if (!set_angle_cli_.exists()) {
        set_angle_cli_.waitForExistence(ros::Duration(0.5));
    }

    // Gọi lần lượt kênh 0..4 với góc (độ)
    const float vals[5] = { sc.ch0, sc.ch1, sc.ch2, sc.ch3, sc.ch4 };
    for (uint8_t ch = 0; ch < 5; ++ch)
    {
        pca9685_servo_control::SetAngle srv;
        srv.request.channel   = ch;
        srv.request.angle_deg = vals[ch];

		ROS_INFO("Setting servo ch=%u to %.1f deg", ch, vals[ch]);

        if (set_angle_cli_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Servo ch=%u -> %.1f deg OK: %s", ch, vals[ch], srv.response.message.c_str());
            } else {
                ROS_WARN("Servo ch=%u -> %.1f deg FAILED: %s", ch, vals[ch], srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed calling /pca9685_servo/set_angle (ch=%u, deg=%.1f)", ch, vals[ch]);
        }
    }
}

/* ************************* Function RC Control ***********************************/

// decode implementation for RC channels (placed in .cpp so uavlink types/macros are available)
static inline void uavlink_rc_channels_decode(const uavlink_message_t *msg, uavlink_rc_channels_t *rc)
{
    if (!msg || !rc) return;
    uint8_t len = msg->len < UAVLINK_MSG_ID_RC_CHANNELS_LEN ? msg->len : UAVLINK_MSG_ID_RC_CHANNELS_LEN;
    memset(rc, 0, UAVLINK_MSG_ID_RC_CHANNELS_LEN);
    memcpy(rc, _MAV_PAYLOAD(msg), len);
}

// Handler cho RC channels (uavlink -> mavros OverrideRCIn)
void handle_msg_rc_channels(uavlink_message_t message)
{
    uavlink_rc_channels_t rc;
    uavlink_rc_channels_decode(&message, &rc);

    // mavros_msgs::OverrideRCIn uses fixed-size boost::array<unsigned short, 18>
    // Gán 8 channel đầu, phần còn lại set = 0 (không override)
    // Note: MAVROS expects values in PWM (~1000-2000). Map if needed.
    rc_msg.channels[0] = rc.chan1;
    rc_msg.channels[1] = rc.chan2;
    rc_msg.channels[2] = rc.chan3;
    rc_msg.channels[3] = rc.chan4;
    rc_msg.channels[4] = rc.chan5;
    rc_msg.channels[5] = rc.chan6;
    rc_msg.channels[6] = rc.chan7;
    rc_msg.channels[7] = rc.chan8;
    // zero remaining channels
    for (size_t i = 8; i < rc_msg.channels.size(); ++i)
    {
        rc_msg.channels[i] = 0;
    }

    // publish to MAVROS
    rc_override_pub.publish(rc_msg);

    // ROS_INFO("[RC] Override published ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u ch7=%u ch8=%u",
    //          (unsigned)rc_msg.channels[0], (unsigned)rc_msg.channels[1],
    //          (unsigned)rc_msg.channels[2], (unsigned)rc_msg.channels[3],
    //          (unsigned)rc_msg.channels[4], (unsigned)rc_msg.channels[5],
    //          (unsigned)rc_msg.channels[6], (unsigned)rc_msg.channels[7]);
}

/* ************************* Function Drone Status ***********************************/
// Function send drone status
void send_drone_status()
{
	uavlink_drone_status_t status;
    status.altitude = local_z;  // Lấy độ cao từ biến toàn cục local_z
    status.battery = battery_remaining_calculate(battery_msg.voltage);
    status.latitude = global_msg.latitude;
    status.longitude = global_msg.longitude;

    status.pos_x = local_x;
    status.pos_y = local_y;
    status.pos_z = local_z;

	status.vx = velocity_msg.twist.linear.x;
	status.vy = velocity_msg.twist.linear.y;
	status.vz = velocity_msg.twist.linear.z;

	// Lấy roll, pitch, yaw từ imu_msg
    tf::Quaternion q(
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z,
        imu_msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    status.roll = roll;
    status.pitch = pitch;
    status.yaw = yaw;

    uavlink_message_t msg;
    uavlink_drone_status_encode(&msg, &status);

    char buf[128];
    uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
    writeSocketMessage(buf, len);

    // Debug log
    // ROS_INFO("[DEBUG] Drone status sent: Alt=%.2f, Bat=%d%%, Lat=%.7f, Lon=%.7f, X=%.2f, Y=%.2f, Z=%.2f, Vx=%.2f, Vy=%.2f, Vz=%.2f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
    //     status.altitude, status.battery, status.latitude, status.longitude,
    //     status.pos_x, status.pos_y, status.pos_z,
    //     status.vx, status.vy, status.vz,
    //     status.roll, status.pitch, status.yaw);
}

// Get altitude from uavpose_msg
void handleLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	local_x = msg->pose.position.x;  // lưu giá trị x
	local_y = msg->pose.position.y;  // lưu giá trị y
    local_z = msg->pose.position.z;  // lưu giá trị z
}
// Thêm hàm callback cho timer gửi drone status
void drone_status_timer_cb(const ros::TimerEvent&)
{
    send_drone_status();
}

// Velocity
void handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocity_msg = *msg;
}

// Orientation
void handleImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg = *msg;
}
/*************************************************************************************************/

// Handle waypoint message
void handle_msg_waypoint(uavlink_message_t message)
{
	uavlink_msg_waypoint_t waypoint;
	uavlink_waypoint_decode(&message, &waypoint);
	// ROS_INFO("msg rev:x= %f,y=%f,z=%f",waypoint.targetX,waypoint.targetY,waypoint.targetZ);
	if (waypoint.type == 0)
		waypoint_indoor_vector.push_back(waypoint);
	else if (waypoint.type == 1)
		waypoint_GPS_vector.push_back(waypoint);
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
	ros::Rate r(2);
	uavlink_global_position_int_t global_pos;
	global_pos.vx = (int16_t)(o.twist.twist.linear.x * 100);
	global_pos.vy = (int16_t)(o.twist.twist.linear.y * 100);
	global_pos.vz = (int16_t)(o.twist.twist.linear.z * 100);

	// get data from global_position
	global_pos.alt = (int16_t)(o.pose.pose.position.z * 100);
	// global_pos.alt = 1;
	global_pos.lat = (int32_t)(global_msg.latitude * 10000000);
	global_pos.lon = (int32_t)(global_msg.longitude * 10000000);
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
	check_busy = true;
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
			if (navigate_to_GPS(waypoint_GPS_vector[0], 0.1))
			{
				waypoint_GPS_vector.erase(waypoint_GPS_vector.begin());
			}
		}
	}
	else
	{
		ROS_ERROR("Error: invalid type fly : %d", type_fly);
	}

	check_busy = false;
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

			case UAVLINK_MSG_ID_POSITION_CONTROL:  // Position in Offboard Mode
        		handle_msg_position_control(message);
        		break;

			case UAVLINK_MSG_ID_SERVO_CONTROL:
				// handle_msg_servo_control(message); not used
				handle_msg_servo_channels(message);
				break;

			case UAVLINK_MSG_ID_RC_CHANNELS: // message RC
                handle_msg_rc_channels(message);
                break;

			case UAVLINK_MSG_ID_COMMAND:
				handle_command(message);
				break;

			case UAVLINK_MSG_ID_VELOCITY_CONTROL:
				handle_msg_velocity_control(message);
				break;

			case UAVLINK_CONTROL_ROBOT_MSG_ID:
				handle_msg_control_robot(message);
				break;

			case UAVLINK_MSG_ID_WAYPOINT:
				handle_msg_waypoint(message);
				break;

			default:
				ROS_WARN("Unknown message ID: %d", message.msgid);
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
	}
}
// safety check for position active in Offboard
void check_position_cmd_timeout(const ros::TimerEvent& e)
{
    if (position_control_active && (ros::Time::now() - last_position_cmd_time > position_cmd_timeout)) {
        ROS_WARN("Position command timeout. Disabling position control.");
        handle_cmd_position_control_mode(false);
        
        // Gửi feedback về timeout
        send_position_feedback(false, 0, 0, 0);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UdpSocket");
	ros::NodeHandle nh, nh_priv("~");
	//servo control
	server.initServoBridge(nh);

	// param
	nh_priv.param("port", port, 12345);

	// altitude hold param
	nh_priv.param("alt_hold/enable",   alt_hold_enable, true);
	nh_priv.param("alt_hold/target_z", alt_target_z,    0.5);
	nh_priv.param("alt_hold/kp",       kp_z,            1.0);
	nh_priv.param("alt_hold/kd",       kd_z,            0.6);
	nh_priv.param("alt_hold/max_vz",   max_z_vel,       0.5);
	nh_priv.param("alt_hold/invert_z", invert_z_sign,   false);
	// ROS_INFO("[UdpServer] AltHold: en=%s z=%.2f kp=%.2f kd=%.2f max_vz=%.2f invz=%s",
	// 		alt_hold_enable ? "true":"false", alt_target_z, kp_z, kd_z, max_z_vel, invert_z_sign ? "true":"false");

	// vx override params (private ns: ~vx_override/...)
	nh_priv.param("vx_override/enable", vx_override_enable, false);
	nh_priv.param("vx_override/value",  vx_override_value,  0.05);
	ROS_INFO("[UdpServer] vx_override: en=%s vx=%.3f",
			vx_override_enable ? "true":"false", vx_override_value);

	// dataset logger toggle service
	dataset_toggle_srv = nh.serviceClient<std_srvs::SetBool>("/dataset_logger/toggle");

	// Initial publisher
	manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
	control_robot_pub = nh.advertise<uavlab411::control_robot_msg>("uavlab411/control_robot", 1);
	rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1); // register publisher RC override

	// position pub in offboard
	position_control_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	// velocity pubs (Clover/MAVROS)
	vel_cmd_pub_stamped   = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
	vel_cmd_pub_unstamped = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

	// Initial subscribe
	auto state_sub = nh.subscribe("mavros/state", 1, &handleState);
	auto global_position_sub = nh.subscribe("/mavros/global_position/global", 1, &handleGlobalPosition);
	auto local_position_sub = nh.subscribe("/mavros/global_position/local", 1, &handleLocalPosition);
	auto battery_sub = nh.subscribe("/mavros/battery", 1, &handle_Battery_State);
	auto uavpose_sub = nh.subscribe("uavlab411/uavpose", 1, &handleUavPose);
	// Altitude
	ros::Subscriber local_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, handleLocalPose);
	// Velocity
	auto velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 1, handleVelocity);
	// Orientation
    auto imu_sub = nh.subscribe("/mavros/imu/data", 1, handleImu);

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
	// POSITION TIMER In offboard
	ros::Timer position_timeout_timer = nh.createTimer(ros::Duration(0.1), check_position_cmd_timeout);
	// Timer send drone status
	ros::Timer drone_status_timer = nh.createTimer(ros::Duration(0.5), drone_status_timer_cb);
	
	init();
	ros::spin();
}
