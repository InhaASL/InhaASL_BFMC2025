#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GlobalPosePublisher
{
public:
    GlobalPosePublisher()
    {
        ros::NodeHandle nh("~");

        // 초기 위치 파라미터
        nh.param("initial_x", initial_x_, 0.0);
        nh.param("initial_y", initial_y_, 0.0);
        nh.param("initial_z", initial_z_, 0.0);

        sub_ = nh.subscribe("/ov_msckf/poseimu", 100, &GlobalPosePublisher::poseCallback, this);
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped global_pose_msg;
        global_pose_msg.header.stamp = msg->header.stamp;
        global_pose_msg.header.frame_id = "map";

        // 위치 보정
        global_pose_msg.pose.position.x = initial_x_ + msg->pose.pose.position.x;
        global_pose_msg.pose.position.y = initial_y_ + msg->pose.pose.position.y;
        global_pose_msg.pose.position.z = initial_z_ + msg->pose.pose.position.z;

        // orientation 보정
        tf2::Quaternion q_raw;
        tf2::fromMsg(msg->pose.pose.orientation, q_raw);

        // 보정용 회전: x축 +90도, z축 +90도
        tf2::Quaternion q_x90, q_z90;
        q_x90.setRPY(M_PI/2, 0, 0);
        q_z90.setRPY(0, 0, M_PI/2);

        tf2::Quaternion q_correction = q_x90 * q_z90; // 오른쪽부터 먼저 적용됨

        tf2::Quaternion q_corrected = q_raw * q_correction;
        q_corrected.normalize();  // 정규화는 항상 안전하게

        global_pose_msg.pose.orientation = tf2::toMsg(q_corrected);

        pub_.publish(global_pose_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    double initial_x_, initial_y_, initial_z_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_pose_publisher");
    GlobalPosePublisher node;
    ros::spin();
    return 0;
}
