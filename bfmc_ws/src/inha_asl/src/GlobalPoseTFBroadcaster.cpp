#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

class GlobalPoseTFBroadcaster
{
public:
    GlobalPoseTFBroadcaster()
    {
        ros::NodeHandle nh;
        sub_pose_ = nh.subscribe("/global_pose", 10, &GlobalPoseTFBroadcaster::poseCallback, this);
    }

private:
    ros::Subscriber sub_pose_;
    tf2_ros::TransformBroadcaster br_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tf_msg;

        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.x = msg->pose.position.x;
        tf_msg.transform.translation.y = msg->pose.position.y;
        tf_msg.transform.translation.z = msg->pose.position.z;

        tf_msg.transform.rotation = msg->pose.orientation;

        br_.sendTransform(tf_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_pose_tf_broadcaster");
    GlobalPoseTFBroadcaster node;
    ros::spin();
    return 0;
}
