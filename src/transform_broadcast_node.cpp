#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <turtlesim/Pose.h>

//#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

//#include <tf2/transform_storage.h>
//std::string turtle_name;
std::string node_name;
std::string pose_msg_name;
std::string body_frame;
const tf::Vector3 lidar_offset;
const nav_msgs::Odometry::ConstPtr lidar_pose;

tf::Transform lidar2body;


//boost::shared_ptr <tf::TransformBroadcaster> br;



void poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){

static tf::TransformBroadcaster br;
tf::Transform body2nav;
tf::Quaternion q;
//tf::Quaternion q2;

body2nav.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                                 odom_msg->pose.pose.position.y,
                                 odom_msg->pose.pose.position.z));
 q.setRPY(odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y,
          odom_msg->pose.pose.orientation.z);

 body2nav.setRotation(q);

  br.sendTransform(tf::StampedTransform(body2nav, ros::Time::now(), "nav_frame", "body_frame"));

}

void lidarPoseCallback(const geometry_msgs::Pose::ConstPtr& lidar_pose_msg){

    tf::TransformBroadcaster br;
    tf::Quaternion q;


    lidar2body.setOrigin(tf::Vector3(lidar_pose_msg->position.x,
                                     lidar_pose_msg->position.y,
                                     lidar_pose_msg->position.z));

    q.setRPY(lidar_pose_msg->orientation.x,
             lidar_pose_msg->orientation.y,
             lidar_pose_msg->orientation.z);

    lidar2body.setRotation(q);
 //lidar2body.setOrigin(lidar_offset);
 //lidar2body.setRotation(tf::Quaternion(lidar_pose_msg->orientation.z,lidar_pose_msg->orientation.y,lidar_pose_msg->orientation.x));
    std::cout<< "Transform Send"<<std::endl;
 br.sendTransform(tf::StampedTransform(lidar2body,ros::Time::now(),"body_frame","lidar_frame"));

}

int main(int argc, char** argv){


  ros::init(argc, argv, "Transform_Node");

     ros::NodeHandle node = ros::NodeHandle("~");

  //br.reset(new tf::TransformBroadcaster);


  if(node.getParam("pose_message_name",pose_msg_name)){
      ROS_INFO("Got Param: %s",pose_msg_name.c_str());

  }else{
      pose_msg_name = "/ArduPilot/pose_data";
      ROS_WARN("Failed to get Pose subscribe message name from Launch File, defaulting to %s \n",pose_msg_name.c_str());
  }

 if(node.getParam("base_frame_id",body_frame)){

 }else{
    body_frame = "body_frame";
 }

  /*if(node.getParam("lidar_offset",lidar_offset)){

  }else{
      lidar_offset = tf::Vector3(0,0,0);
  }*/

 // node.getParam("lidar_frame_id",lidar_frame);

  ros::Subscriber sub = node.subscribe(pose_msg_name, 10, &poseCallback);
  //ros::Subscriber lidar_sub = node.subscribe(lidar_msg_name,10,&lidarPoseCallBack);
  ros::spin();
  return 0;
}
