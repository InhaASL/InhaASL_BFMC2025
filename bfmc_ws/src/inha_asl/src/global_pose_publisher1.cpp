#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//cmh global pose

class GlobalPosePublisher
{
public:
    GlobalPosePublisher()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // 초기 offset 파라미터 로드 (map 상의 시작 좌표)
        pnh.param("init_x", init_x_, 0.0);
        pnh.param("init_y", init_y_, 0.0);
        pnh.param("init_z", init_z_, 0.0);

        sub_pose_ = nh.subscribe("/ov_msckf/poseimu", 10,
                                 &GlobalPosePublisher::poseCallback, this);
        pub_global_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/global_pose1", 10);

        ROS_INFO("GlobalPosePublisher initialized. Using offset (%.2f, %.2f, %.2f)",
                  init_x_, init_y_, init_z_);
    }

private:
    ros::Subscriber sub_pose_;
    ros::Publisher pub_global_pose_;
    double init_x_, init_y_, init_z_;

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseWithCovarianceStamped global_msg = *msg;

        // cm 단위로 변환 
        // global_msg.pose.pose.position.x *= 100;
        // global_msg.pose.pose.position.y *= 100;
        // global_msg.pose.pose.position.z *= 100;
        // 위치에 offset 더해줌 (map 좌표 기준으로 변환)
        global_msg.pose.pose.position.x += init_x_;
        global_msg.pose.pose.position.y += init_y_;
        global_msg.pose.pose.position.z += init_z_;

        // 헤더는 그대로 유지 (필요하면 frame_id를 "map"으로 바꿔도 됨)
        global_msg.header.frame_id = "map";

        pub_global_pose_.publish(global_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_pose_publisher1");
    GlobalPosePublisher node;
    ros::spin();
    return 0;
}
