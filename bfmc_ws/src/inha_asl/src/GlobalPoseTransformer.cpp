#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

class GlobalPoseTransformer
{
public:
    GlobalPoseTransformer()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~"); // private node handle for params

        // 파라미터에서 초기 위치 읽기 (없으면 기본값)
        double x, y, z, qx, qy, qz, qw;
        pnh.param("init_x", x, 0.0);
        pnh.param("init_y", y, 0.0);
        pnh.param("init_z", z, 0.0);
        pnh.param("init_qx", qx, 0.0);
        pnh.param("init_qy", qy, 0.0);
        pnh.param("init_qz", qz, 0.0);
        pnh.param("init_qw", qw, 1.0);

        // 초기 pose 변환 행렬 생성
        Eigen::Quaterniond q_init(qw, qx, qy, qz);
        Eigen::Matrix3d R_init = q_init.toRotationMatrix();

        init_pose_ = Eigen::Matrix4d::Identity();
        init_pose_.block<3,3>(0,0) = R_init;
        init_pose_.block<3,1>(0,3) << x, y, z;

        has_init_ = true;

        // Subscriber & Publisher
        sub_pose_ = nh.subscribe("/ov_msckf/poseimu", 10, &GlobalPoseTransformer::poseCallback, this);
        pub_global_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);

        ROS_INFO("GlobalPoseTransformer initialized with initial pose: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    }

private:
    ros::Subscriber sub_pose_;
    ros::Publisher pub_global_pose_;

    Eigen::Matrix4d init_pose_;
    bool has_init_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!has_init_)
        {
            ROS_WARN_THROTTLE(5, "Initial pose not set.");
            return;
        }

        Eigen::Quaterniond q_vio(msg->pose.orientation.w,
                                 msg->pose.orientation.x,
                                 msg->pose.orientation.y,
                                 msg->pose.orientation.z);
        Eigen::Matrix3d R_vio = q_vio.toRotationMatrix();

        Eigen::Vector3d t_vio(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        Eigen::Matrix4d T_vio = Eigen::Matrix4d::Identity();
        T_vio.block<3,3>(0,0) = R_vio;
        T_vio.block<3,1>(0,3) = t_vio;

        Eigen::Matrix4d T_global = init_pose_ * T_vio;

        geometry_msgs::PoseStamped global_msg;
        global_msg.header.stamp = ros::Time::now();
        global_msg.header.frame_id = "map";

        global_msg.pose.position.x = T_global(0,3);
        global_msg.pose.position.y = T_global(1,3);
        global_msg.pose.position.z = T_global(2,3);

        Eigen::Matrix3d R_global = T_global.block<3,3>(0,0);
        Eigen::Quaterniond q_global(R_global);

        global_msg.pose.orientation.x = q_global.x();
        global_msg.pose.orientation.y = q_global.y();
        global_msg.pose.orientation.z = q_global.z();
        global_msg.pose.orientation.w = q_global.w();

        pub_global_pose_.publish(global_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_pose_transformer");
    GlobalPoseTransformer node;
    ros::spin();
    return 0;
}
