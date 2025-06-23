#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

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
        origin_transform_.setOrigin(tf2::Vector3(-initial_y_,initial_z_,initial_x_));
        origin_transform_.setRotation(q_init);

        has_initial_pose_ = false;

        sub_ = nh.subscribe("/ov_msckf/poseimu", 100, &GlobalPosePublisher::poseCallback, this);
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        const geometry_msgs::Pose& pose_msg = msg->pose.pose;

        // 메시지 → tf2::Transform
        tf2::Transform relative_tf;
        tf2::fromMsg(pose_msg, relative_tf);

        // tf2::Quaternion q_fix(0.5, 0.5, 0.5, 0.5);
        // q_fix.normalize();  // 안전하게 정규화
        // // 회전자체
        // relative_tf.setRotation(q_fix * relative_tf.getRotation());
        // relative_tf.setOrigin(tf2::quatRotate(q_fix, relative_tf.getOrigin()));

        if (!has_initial_pose_)
        {
            prev_relative_tf_ = relative_tf;
            has_initial_pose_ = true;
            return;
        }

        // 3) 이제 delta 계산
        tf2::Transform delta = prev_relative_tf_.inverseTimes(relative_tf);
        // 4) prev 업데이트
        prev_relative_tf_ = relative_tf;

        // 누적 변화 적용
        accumulated_tf_ = delta * accumulated_tf_;

        // global pose = origin * accumulated
        tf2::Transform global_tf = origin_transform_ * accumulated_tf_;

        // 발행할 메시지 생성
        geometry_msgs::PoseStamped global_pose_msg;
        global_pose_msg.header.stamp = msg->header.stamp;
        global_pose_msg.header.frame_id = "map";

        // tf2::Transform → geometry_msgs::Pose 로 수동 변환
        tf2::Vector3   t = global_tf.getOrigin();
        tf2::Quaternion q = global_tf.getRotation();
        geometry_msgs::Pose pose;
        pose.position.x    = t.z();
        pose.position.y    = -t.x();
        pose.position.z    = -t.y();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        global_pose_msg.pose = pose;
        pub_.publish(global_pose_msg);

        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header = global_pose_msg.header;
        tf_stamped.header.frame_id    = "map";
        tf_stamped.child_frame_id     = "base_link";
        tf_stamped.transform.translation.x = t.z();
        tf_stamped.transform.translation.y = -t.x();
        tf_stamped.transform.translation.z = -t.y();
        tf_stamped.transform.rotation      = global_pose_msg.pose.orientation;

        tf_broadcaster_.sendTransform(tf_stamped);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2::Transform origin_transform_;                                      // 초기 위치
    tf2::Transform accumulated_tf_ = tf2::Transform::getIdentity();         // 누적 상대 변화
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
