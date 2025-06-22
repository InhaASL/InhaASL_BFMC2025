#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class GlobalToBaseLinkTF
{
public:
    GlobalToBaseLinkTF()
    {
        ros::NodeHandle nh;
        sub_pose_ = nh.subscribe("/ov_msckf/poseimu", 10,
                                 &GlobalToBaseLinkTF::poseCallback, this);
        ROS_INFO("TF Broadcaster: Listening to /ov_msckf/poseimu and publishing tf from 'start' to 'base_link'");
    }

private:
    ros::Subscriber sub_pose_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // 1) 원본 포즈 읽기
        const auto& p     = msg->pose.pose.position;
        const auto& q_msg = msg->pose.pose.orientation;

        // 2) TF2 quaternion 으로 변환 및 정규화 후 역변환
        tf2::Quaternion q_meas;
        tf2::fromMsg(q_msg, q_meas);
        q_meas.normalize();
        tf2::Quaternion q_meas_inv = q_meas.inverse();

        // 3) 추가 회전: RPY = (0°, 90°, 0°)
        tf2::Quaternion q_offset;
        // q_offset.setRPY(0.0, M_PI/2.0, 0.0);  // pitch만 +90°
        q_offset.setRPY(-M_PI/2.0, 0, -M_PI/2.0);  //cmp

        q_offset.normalize();

        // 4) 최종 orientation = offset * (q_meas 역변환)
        //    => 보정 회전을 먼저 적용한 뒤 역변환을 씌워 줌
        
        tf2::Quaternion q_final = q_offset * q_meas_inv; // 기존


        q_final.normalize();

        // 5) TF 메시지 구성
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp    = msg->header.stamp;
        tf_msg.header.frame_id = "start";
        tf_msg.child_frame_id  = "base_link";

        double z_offset = 0.1; // 이거 해줄필요없음. 지금 맞음 
        tf_msg.transform.translation.x = p.x;
        tf_msg.transform.translation.y = p.y;
        tf_msg.transform.translation.z = p.z ;
        tf_msg.transform.rotation      = tf2::toMsg(q_final);

        // 6) 전송
        tf_broadcaster_.sendTransform(tf_msg);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_to_base_link_tf_broadcaster");
    GlobalToBaseLinkTF node;
    ros::spin();
    return 0;
}
