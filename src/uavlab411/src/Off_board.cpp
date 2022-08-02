#include "uavlab411/Off_board.h"

OffBoard::OffBoard()
{
    sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &OffBoard::handleState, this);
    sub_uavpose = nh.subscribe<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1, &OffBoard::handlePoses, this);
    sub_local_position = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &OffBoard::handleLocalPosition, this);
    sub_global_position = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &OffBoard::handleGlobalPosition, this);
    sub_imu_data = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &OffBoard::handleImuData, this);

    pub_navMessage = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 20);
    pub_pointMessage = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    pub_globalMessage = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 20);

    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    navigate_srv = nh.advertiseService("uavlab411/navigate", &OffBoard::Navigate, this);
    navigateGlobal_srv = nh.advertiseService("uavlab411/navigate_global", &OffBoard::NavigateGlobal, this);
    pid_tuning_srv = nh.advertiseService("uavlab411/pid_tuning", &OffBoard::TuningPID, this);
    takeoff_srv = nh.advertiseService("uavlab411/takeoff", &OffBoard::TakeoffSrv, this);
    land_srv = nh.advertiseService("uavlab411/land", &OffBoard::Land, this);
    telemetry_srv = nh.advertiseService("uavlab411/telemetry", &OffBoard::GetTelemetry, this);

    _uavpose_timemout = ros::Duration(nh.param("uavpose_timeout", 2.0));
    _uavpose_local_position_timeout = ros::Duration(nh.param("rangefinder_timeout", 3.0));
    _land_timeout = ros::Duration(nh.param("land_timeout", 3.0));
    // Default PID
    Kp_yaw = 1;
    Ki_yaw = 0.2;
    Kd_yaw = 0;
    // targetZ = 0;
    Kp_vx = 0.5;
    Ki_vx = 0.01;
    Kd_vx = 0;
    Kp_vz = 1;
    // Default state hold mode
    _curMode = Hold;
    tolerance = 0.1;
    // Initial value
    update_frequency = 35.0;
    _navMessage.coordinate_frame = PositionTarget::FRAME_BODY_NED;
    setpoint_timer = nh.createTimer(ros::Duration(1.0 / update_frequency),
                                    boost::bind(&OffBoard::publish_point, this),
                                    false, false);
}

void OffBoard::handleState(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

void OffBoard::handlePoses(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _uavpose = *msg;
}

void OffBoard::handleLocalPosition(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _uavpose_local_position = *msg;
}

void OffBoard::handleGlobalPosition(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    _globalPos = *msg;
}

void OffBoard::handleImuData(const sensor_msgs::Imu::ConstPtr &msg)
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
void OffBoard::offboardAndArm()
{
    ros::Rate r(10);

    if (cur_state.mode != "OFFBOARD")
    {
        auto start = ros::Time::now();
        ROS_INFO("switch to OFFBOARD");
        static mavros_msgs::SetMode sm;
        sm.request.custom_mode = "OFFBOARD";

        if (!srv_set_mode.call(sm))
            throw std::runtime_error("Error calling set_mode service");

        // wait for OFFBOARD mode
        while (ros::ok())
        {
            ros::spinOnce();
            if (cur_state.mode == "OFFBOARD")
            {
                break;
            }
            else if (ros::Time::now() - start > ros::Duration(5))
            {
                throw std::runtime_error("Offboard timeout!");
            }
            ros::spinOnce();
            r.sleep();
        }
    }

    if (!cur_state.armed)
    {
        z_map = _uavpose_local_position.pose.position.z;
        ROS_INFO("ZMAP: %f", z_map);
        ros::Time start = ros::Time::now();
        ROS_INFO("arming");
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (!srv_arming.call(srv))
        {
            throw std::runtime_error("Error calling arming service");
        }

        // wait until armed
        while (ros::ok())
        {
            ros::spinOnce();
            if (cur_state.armed)
            {
                break;
            }
            else if (ros::Time::now() - start > ros::Duration(5))
            {
                throw std::runtime_error("Arming timed out");
            }
            ros::spinOnce();
            r.sleep();
        }
    }
}

void OffBoard::publish_point()
{
    switch (_curMode)
    {
    case NavYaw: // navigate to waypoint with yawrate mode
        if (!TIMEOUT(_uavpose, _uavpose_timemout))
        {
            navToWaypoint(targetX, targetY, targetZ, update_frequency);
            _navMessage.header.stamp = ros::Time::now();
            pub_navMessage.publish(_navMessage);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    case NavNoYaw: // navigate to waypoint without yawrate mode
        if (!TIMEOUT(_uavpose, _uavpose_timemout))
        {
            navToWayPointV2(targetX, targetY, targetZ, update_frequency);
            _navMessage.header.stamp = ros::Time::now();
            pub_navMessage.publish(_navMessage);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    case NavGlobal:
        navToGPSPoint(ros::Time::now(), this->speed);
        pub_navMessage.publish(_navMessage);
        break;
    case Hold: // Hold mode
        holdMode();
        break;
    case Takeoff: // Takeoff mode
        if (!TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout))
        {
            if (_uavpose.pose.position.z > _pointMessage.pose.position.z - tolerance - z_map)
            {
                getCurrentPosition();
                _curMode = Hold;
                ROS_INFO("Switch to HOLD MODE!");
            }
            else
                pub_pointMessage.publish(_pointMessage);
        }
        else
        {
            getCurrentPosition();
            _curMode = Hold;
            ROS_INFO("Switch to HOLD MODE!");
        }
        break;
    default:
        break;
    }
}

void OffBoard::holdMode()
{
    pub_pointMessage.publish(_pointMessage);
}

void OffBoard::getCurrentPosition()
{
    _pointMessage.header.stamp = ros::Time::now();
    _pointMessage.pose.position = _uavpose_local_position.pose.position;
    _pointMessage.pose.orientation = _uavpose_local_position.pose.orientation;
}

void OffBoard::navToWaypoint(float x, float y, float z, int rate)
{
    float _targetYaw;
    float _targetVx;
    float _targetVz;

    _targetYaw = PidControl_yaw(_uavpose.pose.position.x, _uavpose.pose.position.y,
                                x, y, _uavpose.pose.orientation.z, 1.0 / rate);
    _targetVx = PidControl_vx(_uavpose.pose.position.x, _uavpose.pose.position.y,
                              x, y, 1.0 / rate);
    // _targetVz = Control_vz(_uavpose.pose.position.z, z);
    _navMessage.yaw_rate = _targetYaw * 0.8;
    _navMessage.velocity.x = _targetVx;
    // _navMessage.velocity.z = _targetVz;
    _navMessage.position.z = z_map + z;

    if (_targetYaw == 0)
    {
        _curMode = Hold;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

void OffBoard::navToWayPointV2(float x, float y, float z, int rate)
{
    float _targetV, Vx, Vy, Vz, e_x, e_y, alpha_g, yaw;
    _targetV = PidControl_vx(_uavpose.pose.position.x, _uavpose.pose.position.y,
                             x, y, 1.0 / rate);
    _targetV = _targetV / 2.0;

    e_x = x - _uavpose.pose.position.x;
    e_y = y - _uavpose.pose.position.y;

    alpha_g = atan2(e_y, e_x);
    yaw = alpha_g - _uavpose.pose.orientation.z;
    yaw = atan2(sin(yaw), cos(yaw));

    Vx = cos(yaw) * _targetV;
    Vy = sin(yaw) * _targetV;

    // Change nav message
    _navMessage.header.stamp = ros::Time::now();
    _navMessage.velocity.x = Vx;
    _navMessage.velocity.y = Vy;
    // _navMessage.position.z = z_map + z;
    if (sqrt(e_x * e_x + e_y * e_y) < tolerance)
    {
        getCurrentPosition();
        _pointMessage.pose.position.z = z_map + z;
        _curMode = Hold;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

void OffBoard::navToGPSPoint(const ros::Time &stamp, float speed)
{
    double x_src = _globalPos.latitude / 180 * PI;
    double y_src = _globalPos.longitude / 180 * PI;
    double x_dst = _endGPoint.x / 180 * PI;
    double y_dst = _endGPoint.y / 180 * PI;
    double y = sin(y_dst - y_src) * cos(x_dst);
    double x = cos(x_src) * sin(x_dst) - sin(x_src) * cos(x_dst) * cos(y_dst - y_src);
    double azimuth = PI / 2 - atan2(y, x) - yaw_compass;
    azimuth = azimuth > PI ? azimuth - 2 * PI : azimuth < -PI ? azimuth + 2 * PI
                                                              : azimuth;
    double distance = hypot(_globalPos.latitude - _endGPoint.x, _globalPos.longitude - _endGPoint.y) * 1.113195e5;
    _navMessage.header.stamp = ros::Time::now();
    double v = distance * Kp_vx > speed ? speed : distance * Kp_vx;
    _navMessage.velocity.x = v * cos(azimuth);
    _navMessage.velocity.y = v * sin(azimuth);
    _navMessage.velocity.z = v* (_endGPoint.z - _uavpose_local_position.pose.position.z + z_map)/distance * Kp_vz;

    if (distance < tolerance)
    {
        getCurrentPosition();
        _pointMessage.pose.position.z = z_map + _endGPoint.z;
        _curMode = Hold;
        ROS_INFO("Switch to HOLD MODE!");
    }
}

bool OffBoard::GetTelemetry(uavlab411::Telemetry::Request &req, uavlab411::Telemetry::Response &res)
{
    res.x = NAN;
    res.y = NAN;
    res.z = NAN;
    res.voltage = NAN;
    res.cell_voltage = NAN;

    res.mode = _curMode;
    if (!TIMEOUT(_uavpose, _uavpose_timemout))
        res.isPose = true;
    else
        res.isPose = false;
    res.x = _uavpose.pose.position.x;
    res.y = _uavpose.pose.position.y;
    res.z = _uavpose.pose.position.z;
    res.z_map = z_map;

    return true;
}

bool OffBoard::Navigate(uavlab411::Navigate::Request &req, uavlab411::Navigate::Response &res)
{
    if (!TIMEOUT(_uavpose, _uavpose_timemout) && !TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout) && checkState())
    {
        tolerance = req.tolerance == 0 ? tolerance : req.tolerance;
        switch (req.nav_mode)
        {
        case NavYaw:
            ROS_INFO("NAV TO WP WITH YAW CHANGED");
            Ei_yaw = 0;
            Ei_vx = 0;
            _curMode = NavYaw;
            _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                    PositionTarget::IGNORE_PY +
                                    PositionTarget::IGNORE_PZ +
                                    PositionTarget::IGNORE_AFX +
                                    PositionTarget::IGNORE_AFY +
                                    PositionTarget::IGNORE_AFZ +
                                    PositionTarget::IGNORE_YAW;

            // _navMessage.position.z = req.z;
            targetX = req.x;
            targetY = req.y;
            targetZ = req.z;
            res.success = true;
            res.message = "NAVIGATE TO WAYPOINT!";
            return true;
            break;

        case NavNoYaw:
            ROS_INFO("NAV TO WP WITHOUT YAW CHANGED");
            Ei_vx = 0;
            _curMode = NavNoYaw;
            _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                    PositionTarget::IGNORE_PY +
                                    PositionTarget::IGNORE_PZ +
                                    PositionTarget::IGNORE_AFX +
                                    PositionTarget::IGNORE_AFY +
                                    PositionTarget::IGNORE_AFZ +
                                    PositionTarget::IGNORE_YAW;

            // _navMessage.position.z = req.z;
            targetX = req.x;
            targetY = req.y;
            targetZ = req.z;
            res.success = true;
            res.message = "NAVIGATE TO WAYPOINT!";
            return true;
            break;

        default:
            res.message = "DONT KNOW THIS NAV MODE!";
            res.success = false;
            return true;
            break;
        }
    }
    else
    {
        res.message = "NO LOCAL POSITION OR/AND NO RANGE FINDER OR/AND NO OFFB MODE - NO ARM, PLS CHECK!";
        res.success = false;
        return true;
    }
}

bool OffBoard::NavigateGlobal(uavlab411::NavigateGlobal::Request &req,
                              uavlab411::NavigateGlobal::Response &res)
{
    if (checkState())
    {
        tolerance = req.tolerance == 0 ? tolerance : req.tolerance;
        _endGPoint.x = req.lat;
        _endGPoint.y = req.lon;
        _endGPoint.z = req.alt;
        _navMessage.type_mask = PositionTarget::IGNORE_PX +
                                PositionTarget::IGNORE_PY +
                                PositionTarget::IGNORE_PZ +
                                PositionTarget::IGNORE_AFX +
                                PositionTarget::IGNORE_AFY +
                                PositionTarget::IGNORE_AFZ +
                                PositionTarget::IGNORE_YAW;
        speed = req.speed;
        getTime = ros::Time::now();
        _curMode = NavGlobal;
        res.success = true;
        res.message = "Navigate to GPS point";
        ROS_INFO("NAV TO GPS WAYPOINT WITHOUT YAW CHANGED");
    }
    else
    {
        res.success = false;
        res.message = "Takeoff first";
    }
    return true;
}

bool OffBoard::TakeoffSrv(uavlab411::Takeoff::Request &req, uavlab411::Takeoff::Response &res)
{
    ROS_INFO("TAKE OFF MODE");
    publish_point();
    setpoint_timer.start();
    tolerance = req.tolerance == 0 ? tolerance : req.tolerance;
    if (!TIMEOUT(_uavpose_local_position, _uavpose_local_position_timeout))
    {
        offboardAndArm();
        if (checkState())
        {
            _pointMessage.pose.position.z = req.z + z_map;
            _pointMessage.pose.position.x = _uavpose_local_position.pose.position.x;
            _pointMessage.pose.position.y = _uavpose_local_position.pose.position.y;
            _pointMessage.pose.orientation = _uavpose_local_position.pose.orientation;
            // targetZ = req.z + _uavpose.pose.position.z;
            _curMode = Takeoff;
            res.success = true;
            res.message = "TAKE OFF MODE!";
            return true;
        }
    }
    else
    {
        res.success = false;
        res.message = "NO LOCAL POSITION, CANT TAKEOFF!";
        return true;
    }
    return true;
}

bool OffBoard::Land(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    try
    {
        static mavros_msgs::SetMode sm;
        sm.request.custom_mode = "AUTO.LAND";

        if (!srv_set_mode.call(sm))
            throw std::runtime_error("Can't call set_mode service");

        if (!sm.response.mode_sent)
            throw std::runtime_error("Can't send set_mode request");

        static ros::Rate r(10);
        auto start = ros::Time::now();
        while (ros::ok())
        {
            if (cur_state.mode == "AUTO.LAND")
            {
                res.success = true;
                return true;
            }
            if (ros::Time::now() - start > _land_timeout)
                throw std::runtime_error("Land request timed out");

            ros::spinOnce();
            r.sleep();
        }
    }
    catch (const std::exception &e)
    {
        res.message = e.what();
        ROS_INFO("%s", e.what());
        return true;
    }
    return false;
}

bool OffBoard::TuningPID(uavlab411::PidTuning::Request &req, uavlab411::PidTuning::Response &res)
{
    switch (req.type)
    {
    case 0: // Tuning yawrate PID
        // Response last PID Param
        res.Kp = Kp_yaw;
        res.Ki = Ki_yaw;
        res.Kd = Kd_yaw;
        // New PID param
        Kp_yaw = req.Kp;
        Ki_yaw = req.Ki;
        Kd_yaw = req.Kd;

        res.success = true;
        break;
    case 1: // Tuning velocity PID x
        // Response last PID Param
        res.Kp = Kp_vx;
        res.Ki = Ki_vx;
        res.Kd = Kd_vx;
        // New PID param
        Kp_vx = req.Kp;
        Ki_vx = req.Ki;
        Kd_vx = req.Kd;

        res.success = true;
        break;
    case 2: // Tuning velocity PID z
        // Response last PID Param
        res.Kp = Kp_vz;
        res.Ki = 0;
        res.Kd = 0;
        // New PID param
        Kp_vz = req.Kp;
        Ki_vz = 0;
        Kd_vz = 0;
        res.success = true;
        break;
    default:
        res.success = false;
        return false;
        break;
    }
    return true;
}

float OffBoard::PidControl_yaw(float x_cur, float y_cur, float x_goal, float y_goal, float alpha, float dt)
{

    float e_x;
    float e_y;
    float Error_pre = Error_yaw;
    float alpha_g;
    float Ed_yaw;
    float w;
    // Calculate distance from UAV to goal
    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    // If distance tolen < 0.05m -> return
    if (abs(e_x) < 0.1 && abs(e_y) < 0.1)
    {
        ROS_INFO("Navigate to waypoint success!");
        return 0;
    }

    // Calculate alpha error between robot and waypoint

    // Angle from robot to waypoint
    alpha_g = atan2(e_y, e_x);

    Error_yaw = alpha_g - alpha;
    Error_yaw = atan2(sin(Error_yaw), cos(Error_yaw));

    Ei_yaw += Error_yaw * dt;
    Ed_yaw = (Error_yaw - Error_pre) / dt;

    // PID Function
    w = Kp_yaw * Error_yaw + Ki_yaw * Ei_yaw + Kd_yaw * Ed_yaw;
    w = w > 2 ? 2 : w < -2 ? -2
                           : w;
    return w;
}

float OffBoard::PidControl_vx(float x_cur, float y_cur, float x_goal, float y_goal, float dt)
{
    float e_x;
    float e_y;
    float Error_pre = Error_vx;
    float Ed_vx;
    float w;
    // Calculate distance from UAV to goal
    e_x = x_goal - x_cur;
    e_y = y_goal - y_cur;

    Error_vx = sqrt(e_x * e_x + e_y * e_y);

    Ei_vx += Error_vx * dt;
    Ed_vx = (Error_vx - Error_pre) / dt;

    // PID Function
    w = Kp_vx * Error_vx + Ki_vx * Ei_vx + Kd_vx * Ed_vx;
    return w;
}

bool OffBoard::checkState()
{
    if (cur_state.mode != "OFFBOARD")
    {
        setpoint_timer.stop();
        throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
        return false;
    }
    else if (!cur_state.armed)
    {
        setpoint_timer.stop();
        throw std::runtime_error("Copter is not armed, use auto_arm?");
        return false;
    }
    else
    {
        return true;
    }
}

void OffBoard::get_Distance_Azimuth(const geometry_msgs::Point &from, const geometry_msgs::Point &to)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ROS_INFO("OFFBOARD NODE INITIAL!");

    OffBoard ob;

    ros::spin();
    return 0;
}
