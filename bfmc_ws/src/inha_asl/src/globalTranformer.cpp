#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
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
        nh.param("initial_yaw", initial_yaw_deg_, 0.0); // degree

        double initial_yaw_rad = initial_yaw_deg_ * M_PI / 180.0;

        tf2::Quaternion q_init;
        q_init.setRPY(0, 0, initial_yaw_rad);
        origin_transform_.setOrigin(tf2::Vector3(initial_x_, initial_y_, initial_z_));
        origin_transform_.setRotation(q_init);

        has_initial_pose_ = false;

        sub_ = nh.subscribe("/ov_msckf/poseimu", 100, &GlobalPosePublisher::poseCallback, this);
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        const geometry_msgs::Pose& pose_msg = msg->pose.pose;

        tf2::Transform relative_tf;
        tf2::fromMsg(pose_msg, relative_tf);

        if (!has_initial_pose_)
        {
            prev_relative_tf_ = relative_tf;
            has_initial_pose_ = true;
            return;
        }

        // 변화량 계산
        tf2::Transform delta = prev_relative_tf_.inverseTimes(relative_tf);
        prev_relative_tf_ = relative_tf;

        // 누적 변화 적용
        accumulated_tf_ = accumulated_tf_ * delta;

        // global pose = origin * accumulated
        tf2::Transform global_tf = origin_transform_ * accumulated_tf_;

        // 발행할 메시지 생성
        geometry_msgs::PoseStamped global_pose_msg;
        global_pose_msg.header.stamp = msg->header.stamp;
        global_pose_msg.header.frame_id = "map";
        global_pose_msg.pose = tf2::toMsg(global_tf);

        pub_.publish(global_pose_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf2::Transform origin_transform_;                    // 초기 위치
    tf2::Transform accumulated_tf_ = tf2::Transform::getIdentity();  // 누적 상대 변화
    tf2::Transform prev_relative_tf_;
    bool has_initial_pose_;

    double initial_x_, initial_y_, initial_z_;
    double initial_yaw_deg_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_pose_publisher");
    GlobalPosePublisher node;
    ros::spin();
    return 0;
}
