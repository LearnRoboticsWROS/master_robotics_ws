#include <iostream>
#include <filesystem>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

void isolateCylinder(const std::string& input_pcd, const std::string& output_pcd)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    cloud_reader.read (input_pcd,*cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.0005,0.0005,0.0005); // PLAY 
    voxel_filter.filter(*cloud_filtered);


    // plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers_plane, *coefficients_plane);

    // Extract the negative planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extraxted;
    neg_plane_extraxted.setInputCloud(cloud_filtered);
    neg_plane_extraxted.setIndices(inliers_plane);
    neg_plane_extraxted.setNegative(true);
    neg_plane_extraxted.filter(*cloud_filtered);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.05); // PLAY --> based on the z direction, check the pcd file
    pass.filter(*cloud_filtered);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 0.2); // PLAY --> based on the x direction, check the pcd file
    pass.filter(*cloud_filtered);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.2, 0); // PLAY --> based on the y direction, check the pcd file
    pass.filter(*cloud_filtered);


    // Remove outliers using a statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // *********** //

    // Segment the cylinder using RANSAC
    // Get the normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normals_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(cloud_filtered);
    normals_estimator.setKSearch(50); // play
    normals_estimator.compute(*cloud_normals);


    // Segmentation of the cylinder from the normals 
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> cylinder_segmentator;
    cylinder_segmentator.setOptimizeCoefficients(true);
    cylinder_segmentator.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentator.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentator.setNormalDistanceWeight(0.1);
    cylinder_segmentator.setMaxIterations(10000);
    cylinder_segmentator.setDistanceThreshold(0.005); // play
    cylinder_segmentator.setRadiusLimits(0.001, 0.02); // play
    cylinder_segmentator.setInputCloud(cloud_filtered);
    cylinder_segmentator.setInputNormals(cloud_normals);

    // Get inliers and model coefficients
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    cylinder_segmentator.segment(*inliers_cylinder,*coefficients_cylinder);


    // Extract the cylinder
    pcl::ExtractIndices<pcl::PointXYZ> cylinder_extracted;
    cylinder_extracted.setInputCloud(cloud_filtered);
    cylinder_extracted.setIndices(inliers_cylinder);
    cylinder_extracted.setNegative(false);


    cylinder_extracted.filter(*cloud_filtered);


    // compute the controid of the extracted cylinder points
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);

    // Print the centroid coordinates
    std::cout << "centroid of the cylinder in camera frame: ("
              << centroid[0] << ", "
              << centroid[1] << ", "
              << centroid[2] << ")" << std::endl;


    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd,*cloud_filtered, false);





}


int main(int argc, char** argv)
{
    std::string path_input="/home/ros/master_ws/src/3D_cv/src/inputs/";
    std::string path_output="/home/ros/master_ws/src/3D_cv/src/outputs/";
    std::string input_pcd = path_input + std::string("test.pcd");
    std::string output_pcd = path_output+std::string("output_points.pcd");

    isolateCylinder(input_pcd, output_pcd);

    return 0;
}
