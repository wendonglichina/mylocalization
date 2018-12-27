#include "odom_torun_map.h"
#include <math.h>

OdomRunMap::OdomRunMap()
{
}
OdomRunMap::~OdomRunMap()
{
}

void OdomRunMap::loadParam(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    nh_ = nh;
    private_nh_ = private_nh;
    //private_nh_.param("rosbag_path", rosbags_path_, std::string("/home/bags/data.bag"));
    //private_nh_.param("fusion_odom_topic", fusion_odom_topic_, std::string("/fusion/odom"));
   // private_nh_.param("ground_truth_odom_topic", ground_truth_odom_topic_, std::string("/velodyne_points"));
    private_nh_.param("gnss_odom_in_map_topic", gnss_odom_in_map_topic_, std::string("/gnss_odom_in_map"));
}

void OdomRunMap::readOdomFromRosbag()
{ 
    // if (access(rosbag_path_.c_str(), 0) != 0)
    // {
    //     std::cout << rosbag_path_ << " not exist!" << std::endl;
    // }
    nav_msgs::OdometryPtr nowOdom;
    tf::TransformBroadcaster *pub_odom_tf = new tf::TransformBroadcaster();

    //ros::Publisher pub_fusion_odom = nh_.advertise<nav_msgs::Odometry>(fusion_odom_topic_, 1);
    //ros::Publisher pub_ground_truth_odom = nh_.advertise<nav_msgs::Odometry>(ground_truth_odom_topic_, 1);
    //ros::Publisher pub_ground_truth_odom = nh_.advertise<sensor_msgs::PointCloud2>(ground_truth_odom_topic_, 2);
    ros::Publisher pub_gnss_odom = nh_.advertise<nav_msgs::Odometry>(gnss_odom_in_map_topic_, 1);

    std::vector<std::string> topics;

    //topics.push_back(fusion_odom_topic_);
    //topics.push_back(ground_truth_odom_topic_);
    topics.push_back(gnss_odom_in_map_topic_);

    //读入rosbag
    rosbag::Bag rosbag;
    rosbag.open(rosbag_path_, rosbag::bagmode::Read);

    //读取bag帧
    rosbag::View view(rosbag, rosbag::TopicQuery(topics));

    if (view.size() == 0)
    {
        std::cout << "Can't find msg topic in " << rosbag_path_ << std::endl;
        exit(-1);
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
    {
        // if (it->getTopic() == fusion_odom_topic_.c_str())
        // {
        //     nav_msgs::OdometryPtr msg = it->instantiate<nav_msgs::Odometry>();
        //     if (msg != NULL)
        //     {
        //         pub_fusion_odom.publish(msg);

        //         //发布TF
        //         geometry_msgs::TransformStamped tf_trans;
        //         tf_trans.header.stamp = msg->header.stamp;
        //         tf_trans.header.frame_id = msg->header.frame_id;
        //         tf_trans.child_frame_id = msg->child_frame_id;
        //         tf_trans.transform.translation.x = msg->pose.pose.position.x;
        //         tf_trans.transform.translation.y = msg->pose.pose.position.y;
        //         tf_trans.transform.translation.z = msg->pose.pose.position.z;
        //         tf_trans.transform.rotation.w = msg->pose.pose.orientation.w;
        //         tf_trans.transform.rotation.x = msg->pose.pose.orientation.x;
        //         tf_trans.transform.rotation.y = msg->pose.pose.orientation.y;
        //         tf_trans.transform.rotation.z = msg->pose.pose.orientation.z;
        //         pub_odom_tf->sendTransform(tf_trans);
        //     }
        // }

        // if (it->getTopic() == ground_truth_odom_topic_.c_str())
        // {
        //     sensor_msgs::PointCloud2ConstPtr msg = it->instantiate<sensor_msgs::PointCloud2>();
            
        //     //nav_msgs::OdometryPtr msg = it->instantiate<nav_msgs::Odometry>();
        //     if (msg != NULL)
        //     {
        //         pub_ground_truth_odom.publish(msg);
        //     }
        // }
        
        if (it->getTopic() == gnss_odom_in_map_topic_.c_str())
        {
            nav_msgs::OdometryPtr msg = it->instantiate<nav_msgs::Odometry>();
            if (msg != NULL)
            {
                if (nowOdom==NULL)
                {
                    nowOdom = msg;
                    continue;
                }
                int lenth = pow(msg->pose.pose.position.x-nowOdom->pose.pose.position.x,2) + pow(msg->pose.pose.position.y-nowOdom->pose.pose.position.y,2);
                if (lenth > 64)
                {
                    nowOdom = msg;
                    pub_gnss_odom.publish(msg);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
    rosbag.close();
}
void OdomRunMap::baglistget()
{
    DIR *dir;
    struct dirent *ptr;
    std::string locpath = "/home/bags/lidar3d_loc/map_cars/";
    std::string rosbag_path;
    private_nh_.param("rosbag_path", rosbags_path_, std::string("/home/bags/data.bag"));

    if (opendir(rosbags_path_.c_str()) == NULL)
    {
        std::cout << "[SubmapStitching]: Open submap dir " << rosbags_path_ << " file error" << std::endl;
        exit(1);
    }
    dir = opendir(rosbags_path_.c_str());
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
        {
            continue;
        }
        else if (ptr->d_type == 8) //file
        {
            //file_set.insert(ptr->d_name);
            rosbag_path_ = ptr->d_name;
            if (rosbag_path_.find(".bag")>0 &&rosbag_path_.find(".bag")<200)
            {
                rosbag_path = rosbags_path_ + rosbag_path_;
                rosbag_path_ = locpath + rosbag_path_;
                std::cout<< rosbag_path<< std::endl;
#if 0
                std::string command_rd = "cp -r " + rosbag_path + " " + rosbag_path_;
                system(command_rd.c_str());
                readOdomFromRosbag();
                command_rd = "rm -r " + rosbag_path_;
                system(command_rd.c_str());
#endif
#if 1
                rosbag_path_ = rosbag_path;
                readOdomFromRosbag();
#endif
            }
            
        }
        else if (ptr->d_type == 10 || ptr->d_type == 4) //link file or dir
        {
           // dir_set.insert(ptr->d_name);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdomRunMap");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    OdomRunMap odom_display;

    odom_display.loadParam(node, private_nh);
    odom_display.baglistget();
    //odom_display.readOdomFromRosbag();

    return 0;
}