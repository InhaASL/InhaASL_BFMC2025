#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <random>
#include <mutex>
#include <Eigen/Dense>
#include <vector>

class AckermannEKF
{
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber drive_sub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer publish_timer_;

    Eigen::VectorXd state_;         // [x, y, theta, v, omega, bax, bay, bgz]
    Eigen::MatrixXd covariance_;    // 8x8 covariance
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_ackermann_;
    Eigen::MatrixXd R_imu_;

    double wheel_base_;
    double driving_gain_, steering_gain_;
    bool use_tf_;
    double publish_rate_;
    std::string odom_topic_, drive_topic_, imu_topic_;
    std::string odom_frame_, base_frame_;

    ros::Time last_predict_time_;
    bool ekf_initialized_;
    std::mutex ekf_mutex_;

    // Bias initialization
    bool bias_initialized_ = false;
    int bias_sample_count_ = 0;
    int bias_sample_limit_ = 200;
    double bax_sum_ = 0.0, bay_sum_ = 0.0, bgz_sum_ = 0.0;

    public:
    AckermannEKF() : state_(8), covariance_(8, 8), Q_(8, 8), 
                     R_ackermann_(2, 2), R_imu_(1, 1),
                     ekf_initialized_(false)
    {
        loadConfiguration();
        initializeEKF();
        setupROS();
        last_predict_time_ = ros::Time::now();
        ROS_INFO("AckermannEKF node started.");
    }

private:
    void loadConfiguration()
    {
        nh_.param("wheel_base", wheel_base_, 0.26);
        nh_.param("use_tf", use_tf_, true);
        nh_.param<std::string>("odom_topic", odom_topic_, "ekf_odom");
        nh_.param<std::string>("drive_topic", drive_topic_, "current_speed");
        nh_.param<std::string>("imu_topic", imu_topic_, "/d455/imu");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param("publish_rate", publish_rate_, 30.0);

        Q_ = Eigen::MatrixXd::Zero(8, 8);
        Q_.block<5,5>(0,0) = Eigen::MatrixXd::Identity(5, 5) * 0.1;
        Q_(5,5) = std::pow(0.0001, 2); // bax
        Q_(6,6) = std::pow(0.0001, 2); // bay
        Q_(7,7) = std::pow(2.5e-6, 2); // bgz

        std::vector<double> ackermann_cov;
        if (nh_.getParam("ackermann_covariance", ackermann_cov) && ackermann_cov.size() == 4)
            R_ackermann_ = Eigen::Map<Eigen::MatrixXd>(ackermann_cov.data(), 2, 2).transpose();
        else
            R_ackermann_ = Eigen::MatrixXd::Identity(2, 2) * 0.2;

        std::vector<double> imu_cov;
        if (nh_.getParam("imu_covariance", imu_cov) && imu_cov.size() == 1)
            R_imu_(0, 0) = imu_cov[0];
        else
            R_imu_(0, 0) = 0.05;

        covariance_ = Eigen::MatrixXd::Identity(8, 8);
        nh_.param("drive_gain", driving_gain_, 9.05);
        nh_.param("steering_gain", steering_gain_, 11.0);
    }

    void setupROS()
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 50);
        drive_sub_ = nh_.subscribe(drive_topic_, 10, &AckermannEKF::driveCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 100, &AckermannEKF::imuCallback, this);
        if (publish_rate_ > 0.0)
            publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &AckermannEKF::publishOdometry, this);
    }

    void initializeEKF()
    {
        state_.setZero();
    }

    void predictStepIMU(double dt, const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (dt <= 0.0 || dt > 0.1) return;

        double x = state_(0), y = state_(1), theta = state_(2);
        double v = state_(3), omega = state_(4);
        double bax = state_(5), bay = state_(6), bgz = state_(7);

        double ax_body = msg->linear_acceleration.x - bax;
        double ay_body = msg->linear_acceleration.y - bay;
        double omega_z = msg->angular_velocity.z - bgz;

        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        double ax_world = cos_theta * ax_body - sin_theta * ay_body;
        double ay_world = sin_theta * ax_body + cos_theta * ay_body;

        Eigen::VectorXd predicted_state = state_;
        predicted_state(0) = x + v * cos_theta * dt + 0.5 * ax_world * dt * dt;
        predicted_state(1) = y + v * sin_theta * dt + 0.5 * ay_world * dt * dt;
        predicted_state(2) = theta + omega_z * dt;
        predicted_state(3) = v + ax_world * dt;
        predicted_state(4) = omega_z;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(8, 8);
        F(0, 2) = -v * sin_theta * dt;
        F(0, 3) = cos_theta * dt;
        F(1, 2) = v * cos_theta * dt;
        F(1, 3) = sin_theta * dt;
        F(2, 4) = dt;

        covariance_ = F * covariance_ * F.transpose() + Q_ * dt;
        state_ = predicted_state;
        state_(2) = normalizeAngle(state_(2));
    }

    void updateAckermann(double v, double steering_angle)
    {
        double omega_ackermann = (v / wheel_base_) * tan(steering_angle) / steering_gain_;
        double v_corrected = v / driving_gain_;

        Eigen::VectorXd z(2);
        z << v_corrected, omega_ackermann;

        Eigen::VectorXd h(2);
        h << state_(3), state_(4);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 8);
        H(0, 3) = 1.0;
        H(1, 4) = 1.0;

        kalmanUpdate(z, h, H, R_ackermann_);
    }

    void updateIMU(const sensor_msgs::Imu::ConstPtr& msg)
    {
        double omega_z = msg->angular_velocity.z;
        Eigen::VectorXd z(1);
        z(0) = omega_z;

        Eigen::VectorXd h(1);
        h(0) = state_(4) + state_(7);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 8);
        H(0, 4) = 1.0;
        H(0, 7) = 1.0;

        kalmanUpdate(z, h, H, R_imu_);
    }

    void kalmanUpdate(const Eigen::VectorXd& z, const Eigen::VectorXd& h,
                      const Eigen::MatrixXd& H, const Eigen::MatrixXd& R)
    {
        Eigen::VectorXd y = z - h;
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();
        state_ += K * y;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
        covariance_ = (I - K * H) * covariance_;
        state_(2) = normalizeAngle(state_(2));
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        ros::Time now = msg->header.stamp;
        if (now.isZero()) now = ros::Time::now();

        if (!ekf_initialized_) {
            last_predict_time_ = now;
            ekf_initialized_ = true;
            return;
        }

        if (!bias_initialized_ && bias_sample_count_ < bias_sample_limit_) {
            bax_sum_ += msg->linear_acceleration.x;
            bay_sum_ += msg->linear_acceleration.y;
            bgz_sum_ += msg->angular_velocity.z;
            bias_sample_count_++;

            if (bias_sample_count_ >= bias_sample_limit_) {
                state_(5) = bax_sum_ / bias_sample_count_;
                state_(6) = bay_sum_ / bias_sample_count_;
                state_(7) = bgz_sum_ / bias_sample_count_;
                bias_initialized_ = true;
                ROS_INFO("IMU bias initialized: bax=%.5f, bay=%.5f, bgz=%.5f", 
                         state_(5), state_(6), state_(7));
            }
            return;
        }

        double dt = (now - last_predict_time_).toSec();
        if (dt > 0.0) {
            predictStepIMU(dt, msg);
            last_predict_time_ = now;
        }

        updateIMU(msg);
    }


    void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        if (!ekf_initialized_) return;
        updateAckermann(msg->drive.speed, -msg->drive.steering_angle);
    }

    void publishOdometry(const ros::TimerEvent&)
    {
        if (!ekf_initialized_) return;
        ros::Time now = ros::Time::now();

        double x = state_(0), y = state_(1), theta = state_(2);
        double v = state_(3), omega = state_(4);
        double vx = v * cos(theta);
        double vy = v * sin(theta);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        if (use_tf_) {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_quat;
            tf_broadcaster_.sendTransform(tf_msg);
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;

        odom.pose.covariance[0] = covariance_(0, 0);
        odom.pose.covariance[7] = covariance_(1, 1);
        odom.pose.covariance[35] = covariance_(2, 2);
        odom.twist.covariance[0] = covariance_(3, 3);
        odom.twist.covariance[7] = covariance_(3, 3);
        odom.twist.covariance[35] = covariance_(4, 4);

        odom_pub_.publish(odom);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ackermann_ekf_odom_node");
    try {
        AckermannEKF ekf_node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Node encountered an error: %s", e.what());
        return -1;
    }
    return 0;
}
