#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h> 
#include <mutex>

class GtsamOptimizer {
public:
    GtsamOptimizer() : isam2(gtsam::ISAM2()), key_counter_x(1), key_counter_y(1), key_counter(1) {}

    void addOdometryFactorX(const gtsam::Pose3& pose) {
        gtsam::Symbol symbol_x('x', key_counter_x++);
        values.insert(symbol_x, pose);
        if (key_counter_x > 1) {  // Ensure there's a previous x to connect to
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_counter_x-2), symbol_x, last_x_pose.between(pose), odometry_noise));
        }
        if (key_counter > 1)
        {
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(last_symbol, symbol_x, last_pose.between(pose), odometry_noise));
        }
        last_x_pose = pose;
        last_pose = pose;
        last_symbol = symbol_x;
        key_counter++;
        return;
    }

    void addOdometryFactorY(const gtsam::Pose3& pose) {
        gtsam::Symbol symbol_y('y', key_counter_y++);
        values.insert(symbol_y, pose);
        if (key_counter_y > 1) {  // Ensure there's a previous x to connect to
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('y', key_counter_y-2), symbol_y, last_y_pose.between(pose), odometry_noise));
        }
        if (key_counter > 1)
        {
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(last_symbol, symbol_y, last_pose.between(pose), odometry_noise));
        }
        last_y_pose = pose;
        last_pose = pose;
        last_symbol = symbol_y;
        key_counter++;
        return;
    }

    void performOptimization() {
        isam2.update(graph);
        graph.resize(0); // Clear the graph after optimization
        return;
    }

    gtsam::Pose3 getOptimizedResult() {
        gtsam::Values result = isam2.calculateEstimate();
        return result.at<gtsam::Pose3>(last_symbol); // Retrieve the pose corresponding to the last symbol
    }
private:
    gtsam::ISAM2 isam2;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    gtsam::Pose3 last_x_pose;
    gtsam::Pose3 last_y_pose;

    gtsam::Symbol last_symbol;
    gtsam::Pose3 last_pose;

    int key_counter_x, key_counter_y, key_counter;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
    gtsam::noiseModel::Diagonal::shared_ptr between_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished());
};

class OdometryNode {
public:
    OdometryNode() : optimizer() {
        ros::NodeHandle nh;

        nh.param<std::string>("config_file", config_file, "/home/wu/自写代码/LC_ws/src/my_odometry_pkg/config/Tcl.yaml");
        // 读取旋转矩阵
        Eigen::Matrix3d rotation_matrix = readRotationMatrix(config_file).inverse();

        sub_odom1 = nh.subscribe("aft_mapped_to_init", 1, &OdometryNode::odom1Callback, this);
        sub_odom2 = nh.subscribe("odom2", 1, &OdometryNode::odom2Callback, this);
        pub_optimized_odom = nh.advertise<nav_msgs::Odometry>("odom_optimized", 5);
    }

    void odom1Callback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        gtsam::Pose3 pose = convertToGtsamPose(msg->pose.pose);
        optimizer.addOdometryFactorX(pose);
        optimizer.performOptimization();
        gtsam::Pose3 optimized_pose = optimizer.getOptimizedResult();
        nav_msgs::Odometry odom_msg = convertToOdometryMsg(optimized_pose, msg->header);
        pub_optimized_odom.publish(odom_msg);
    }

    void odom2Callback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        geometry_msgs::Pose pose_trans;

        // 提取原始位置
        Eigen::Vector3d original_position;
        original_position << msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z;

        // 计算变换后的位置
        Eigen::Vector3d transformed_position = rotation_matrix * original_position;

        // 将变换后的位置赋值给pose_trans
        pose_trans.position.x = transformed_position.x();
        pose_trans.position.y = transformed_position.y();
        pose_trans.position.z = transformed_position.z();

        // 提取原始的四元数并转换为Eigen的Quaterniond
        Eigen::Quaterniond original_orientation(msg->pose.pose.orientation.w,
                                                msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y,
                                                msg->pose.pose.orientation.z);

        // 计算变换后的四元数
        Eigen::Quaterniond transformed_orientation(rotation_matrix * original_orientation.toRotationMatrix());

        // 将变换后的四元数赋值给pose_trans
        pose_trans.orientation.w = transformed_orientation.w();
        pose_trans.orientation.x = transformed_orientation.x();
        pose_trans.orientation.y = transformed_orientation.y();
        pose_trans.orientation.z = transformed_orientation.z();

        gtsam::Pose3 pose = convertToGtsamPose(pose_trans);
        optimizer.addOdometryFactorY(pose);
        optimizer.performOptimization();
        gtsam::Pose3 optimized_pose = optimizer.getOptimizedResult();
        nav_msgs::Odometry odom_msg = convertToOdometryMsg(optimized_pose, msg->header);
        pub_optimized_odom.publish(odom_msg);
    }

private:
    std::string config_file;
    Eigen::Matrix3d rotation_matrix;
    ros::Subscriber sub_odom1;
    ros::Subscriber sub_odom2;
    ros::Publisher pub_optimized_odom;
    GtsamOptimizer optimizer;
    std::mutex mutex_;  // 互斥锁用于保护共享资源

    gtsam::Pose3 convertToGtsamPose(const geometry_msgs::Pose& pose) {
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        gtsam::Point3 trans(pose.position.x, pose.position.y, pose.position.z);
        return gtsam::Pose3(rot, trans);
    }

    nav_msgs::Odometry convertToOdometryMsg(const gtsam::Pose3& pose, const std_msgs::Header& header) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header = header;

        odom_msg.pose.pose.position.x = pose.translation().x();
        odom_msg.pose.pose.position.y = pose.translation().y();
        odom_msg.pose.pose.position.z = pose.translation().z();

        gtsam::Quaternion quat = pose.rotation().toQuaternion();
        odom_msg.pose.pose.orientation.w = quat.w();
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();

        // 这里可以填充其他信息，如速度等，如果有需求的话
        return odom_msg;
    }

    // 函数：从YAML节点中解析旋转矩阵到Eigen::Matrix3d
    Eigen::Matrix3d readRotationMatrix(const std::string& config_file) 
    {
        YAML::Node config = YAML::LoadFile(config_file);

        // 从YAML中读取旋转矩阵
        std::vector<std::vector<double>> matrix = config["rotation_matrix"].as<std::vector<std::vector<double>>>();

        // 将std::vector转换为Eigen::Matrix3d
        Eigen::Matrix3d rotation_matrix;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation_matrix(i, j) = matrix[i][j];
            }
        }
        return rotation_matrix;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_node");
    OdometryNode node;
    ros::spin();
    return 0;
}
