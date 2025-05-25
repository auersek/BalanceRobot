#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct Pose2D {
    double x, y, theta;
    Pose2D(double x_ = 0, double y_ = 0, double theta_ = 0) : x(x_), y(y_), theta(theta_) {}
};

// Apply 2D pose transformation to a point cloud
PointCloudT::Ptr transformPointCloud(const PointCloudT::Ptr& cloud, const Pose2D& pose) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << pose.x, pose.y, 0;
    transform.rotate(Eigen::AngleAxisf(pose.theta, Eigen::Vector3f::UnitZ()));
    
    PointCloudT::Ptr transformed(new PointCloudT);
    pcl::transformPointCloud(*cloud, *transformed, transform);
    return transformed;
}

// Perform ICP between two clouds and return relative transformation
Pose2D performICP(const PointCloudT::Ptr& src, const PointCloudT::Ptr& tgt) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    
    PointCloudT Final;
    icp.align(Final);
    
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    float dx = transformation(0, 3);
    float dy = transformation(1, 3);
    float dtheta = atan2(transformation(1, 0), transformation(0, 0));

    return Pose2D(dx, dy, dtheta);
}

int main(int argc, char** argv) {
    std::vector<std::string> pcd_files = {
        "scan1.pcd", "scan2.pcd", "scan3.pcd" // Replace with your actual Lidar scans
    };

    std::vector<Pose2D> trajectory;
    PointCloudT::Ptr global_map(new PointCloudT);

    Pose2D current_pose(0, 0, 0);
    trajectory.push_back(current_pose);

    PointCloudT::Ptr last_cloud(new PointCloudT);

    for (size_t i = 0; i < pcd_files.size(); ++i) {
        PointCloudT::Ptr current_cloud(new PointCloudT);
        if (pcl::io::loadPCDFile<PointT>(pcd_files[i], *current_cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", pcd_files[i].c_str());
            continue;
        }

        if (i == 0) {
            *global_map += *current_cloud;
            *last_cloud = *current_cloud;
            continue;
        }

        Pose2D relative_pose = performICP(current_cloud, last_cloud);

        // Update pose
        current_pose.x += cos(current_pose.theta) * relative_pose.x - sin(current_pose.theta) * relative_pose.y;
        current_pose.y += sin(current_pose.theta) * relative_pose.x + cos(current_pose.theta) * relative_pose.y;
        current_pose.theta += relative_pose.theta;

        trajectory.push_back(current_pose);

        // Transform and merge
        PointCloudT::Ptr transformed = transformPointCloud(current_cloud, current_pose);
        *global_map += *transformed;

        *last_cloud = *current_cloud;
    }

    // Visualize result
    pcl::visualization::PCLVisualizer viewer("SLAM Map");
    viewer.addPointCloud(global_map, "map");
    viewer.spin();

    return 0;
}
