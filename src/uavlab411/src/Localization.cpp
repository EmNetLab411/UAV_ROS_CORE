#include "uavlab411/Localization.h"

ros::Duration _aruco_timemout = ros::Duration(0.2);
//define global variable
double z_barometer;
geometry_msgs::PoseStamped uav_pose, pose_path,uavpose_of_local_msg;
nav_msgs::Path path_uav;
sensor_msgs::Range _rangefinder;
uint64_t pose_seq = 0;
// define subscriber and publisher
ros::Publisher uavpose_pub, path_pub, uavpose_of_local_pub, pub_baro;

ros::Subscriber main_optical_flow_pose_sub, range_finder_sub;
ros::Subscriber local_position_sub;

//handle subscribe topic funtion
void handle_main_optical_flow_pose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    //convert from main_camera_optical frame_id to aruco_map frame_id
    //main_camera_optical trung goc toa do voi map nhung goc xoay cung chieu voi yaw cua uav
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg.pose.pose.position.x;
    pose2d.y = msg.pose.pose.position.y;
    
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw_aruco;
    m.getRPY(roll, pitch, yaw_aruco);

    double vector_long = sqrt(pose2d.x*pose2d.x + pose2d.y*pose2d.y);
    double alpha = atan(pose2d.x/pose2d.y);
    if (pose2d.y < 0)
    {
        alpha -= PI;
    }
    double x_map,y_map,z_map;
    x_map = vector_long*cos(alpha + yaw_aruco + PI/2);
    y_map = vector_long*sin(alpha + yaw_aruco + PI/2);
    z_map = z_barometer;
    double yaw_map = yaw_aruco + PI/2;
    yaw_map = (yaw_map > PI) ? yaw_map - 2*PI : yaw_map;
    
    //send transfrom from aruco_map frame_id to uav frame_id
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_map,y_map,z_map) );
    tf::Quaternion orientation_map;
    orientation_map.setRPY(0, 0, yaw_map);
    transform.setRotation(orientation_map);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "uav_frame", "aruco_frame"));

    uav_pose.header.frame_id = "aruco_frame";
    uav_pose.header.seq = pose_seq;
    uav_pose.header.stamp = ros::Time::now();
    uav_pose.pose.position.x = x_map;
    uav_pose.pose.position.y = y_map;
    uav_pose.pose.position.z = z_map;

    uav_pose.pose.orientation.z = yaw_map;
    pose_path.header = uav_pose.header;
    pose_path.pose.position = uav_pose.pose.position;
    // quaternionTFToMsg(orientation_map, pose_path.pose.orientation);
    uavpose_pub.publish(uav_pose);
    pose_seq++;

    //Path to visualize on rviz
    path_uav.header = msg.header;
    path_uav.poses.push_back(pose_path);
    path_pub.publish(path_uav);
}


void handle_local_position(const geometry_msgs::PoseStamped& msg)
{
    z_barometer = msg.pose.position.z;
    sensor_msgs::Range range;

    range.min_range = 0;
    range.max_range = 4;
    range.range = z_barometer;
    range.field_of_view = 0.5;
    range.radiation_type = 0;
    pub_baro.publish(range);
    // tf::Quaternion q(
    //     msg.pose.orientation.x,
    //     msg.pose.orientation.y,
    //     msg.pose.orientation.z,
    //     msg.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw_local;
    // m.getRPY(roll, pitch, yaw_local);
    // uavpose_of_local_msg = msg;
    // uavpose_of_local_msg.pose.orientation.w =0;
    // uavpose_of_local_msg.pose.orientation.x =0;
    // uavpose_of_local_msg.pose.orientation.y =0;
    // uavpose_of_local_msg.pose.orientation.z =yaw_local;
    
    // if (TIMEOUT(uav_pose,_aruco_timemout))
    // {
    //     uavpose_of_local_msg.pose.position.x += x_negative;
    //     uavpose_of_local_msg.pose.position.y += y_negative;
    //     uavpose_of_local_msg.pose.orientation.z += yaw_negative;
    //     uavpose_of_local_pub.publish(uavpose_of_local_msg);
    // }
    // else
    // {
    //     x_negative = uav_pose.pose.position.x - msg.pose.position.x;
    //     y_negative = uav_pose.pose.position.y - msg.pose.position.y;
    //     yaw_negative = uav_pose.pose.orientation.z - yaw_local;
    // }
   
}

void handle_rangefinder(const sensor_msgs::RangeConstPtr &range)
{
    _rangefinder = *range;
}
int main(int argc, char **argv)
{
    ros::init(argc,argv, "Navigate_aruco");
    ros::NodeHandle nh, nh_priv("~");

    main_optical_flow_pose_sub = nh.subscribe("/aruco_map/pose",1,&handle_main_optical_flow_pose);
    local_position_sub = nh.subscribe("mavros/local_position/pose",1,&handle_local_position);
    range_finder_sub = nh.subscribe("/rangefinder/range", 1, &handle_rangefinder);
    // Publish
    uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/pathuav", 10);
    pub_baro = nh.advertise<sensor_msgs::Range>("/rangefinder/range", 1);
    uavpose_of_local_pub = nh.advertise<geometry_msgs::PoseStamped>("uavlab411/uavpose", 1);
    ros::spin();
    return 0;
}
