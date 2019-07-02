#include "Gnss_odom_tf.h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <signal.h>

tf::TransformBroadcaster *pub_odom_tf = new tf::TransformBroadcaster();
void start(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    nh_ = nh;
	  private_nh_ = private_nh;
    private_nh_.param("Gnss_topic", Gnss_topic_, std::string("/sensor/nvgi120/odom"));
    gnsstopic = nh_.subscribe(Gnss_topic_, 2, &gnssodom, this);
}
void gnssodom(nav_msgs::OdometryPtr &msg)
{
    geometry_msgs::TransformStamped tf_trans;
    tf_trans.header.stamp = msg->header.stamp;
    tf_trans.header.frame_id = msg->header.frame_id;
    tf_trans.child_frame_id = msg->child_frame_id;
    tf_trans.transform.translation.x = msg->pose.pose.position.x;
    tf_trans.transform.translation.y = msg->pose.pose.position.y;
    tf_trans.transform.translation.z = msg->pose.pose.position.z;
    tf_trans.transform.rotation.w = msg->pose.pose.orientation.w;
    tf_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    tf_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    tf_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    pub_odom_tf->sendTransform(tf_trans);
}
void main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_tf");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
  start(nh,private_nh);
}
