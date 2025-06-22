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
        ros::NodeHandle pnh("~");   // ─ private ns for params
        pnh.param("use_inverse", use_inverse_, true); // 바꿔가며 테스트
        pnh.param("roll_sign",   roll_sign_,  -1);    // 1 또는 -1

        sub_pose_ = nh.subscribe("/ov_msckf/poseimu", 10,
                                 &GlobalToBaseLinkTF::poseCallback, this);
    }

private:
    bool use_inverse_;
    int  roll_sign_;                 // 1  → +90°,  -1 → -90°
    ros::Subscriber sub_pose_;
    tf2_ros::TransformBroadcaster tf_pub_;

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {

        const auto& p = msg->pose.pose.position;
        /* ① 측정 quaternion */
        tf2::Quaternion q_meas;
        tf2::fromMsg(msg->pose.pose.orientation, q_meas);
        q_meas.normalize();
        if (use_inverse_) q_meas = q_meas.inverse();

        /* ② 카메라→base_link 축 보정   (Roll ±90°, Yaw −90°)  */
        tf2::Quaternion q_off;
        q_off.setRPY( roll_sign_ * M_PI/2.0,   // ±90°
                      0.0,
                     -M_PI/2.0 );              // −90°
        q_off.normalize();

        /* ③ 최종 quaternion : 보정 ∘ 측정 */
        tf2::Quaternion q_final = q_off * q_meas;
        q_final.normalize();

        /* ④ TF 메시지 송신 */
        geometry_msgs::TransformStamped tfm;
        tfm.header.stamp    = msg->header.stamp;
        tfm.header.frame_id = "start"; 
        tfm.child_frame_id  = "base_link";
        tfm.transform.translation.x = p.x;
        tfm.transform.translation.y = p.y;
        tfm.transform.translation.z = p.z;
        tfm.transform.rotation    = tf2::toMsg(q_final);
        tf_pub_.sendTransform(tfm);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_to_base_link_tf_broadcaster");
    GlobalToBaseLinkTF node;
    ros::spin();
    return 0;
}
