/*********************************************************************************************************************
Main Function for point cloud registration with ICP point2plane
Last modified: August 24, 2020

Reference:
Chen, Yang, and Gérard Medioni. "Object modelling by registration of multiple range images." 
Image and vision computing 10.3 (1992): 145-155.

Responsible for implementation: ***
Documentation:
**********************************************************************************************************************/
#include "validationlib.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){ 
    //double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source){
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(target);
    
    double fitness_score = 0.0;
    
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    
    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < source->points.size (); ++i){
        //Avoid NaN points as they crash nn searches
        if(!pcl_isfinite((*source)[i].x)){
            continue;
        }
        
        // Find its nearest neighbor in the target
        tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);
        
        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range*max_range){
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }
    
    if (nr > 0){
        //cout << "nr: " << nr << endl;
        //cout << "fitness_score: " << fitness_score << endl;
        return sqrt(fitness_score / nr);
    }else{
        return (std::numeric_limits<double>::max ());
    }
}

int
main (int argc, char** argv)
{
    clock_t tempo;
    tempo = clock();
    PointCloud::Ptr cloud_in (new PointCloud);
    PointCloud::Ptr cloud_out (new PointCloud);
    PointCloud::Ptr cloud_icp (new PointCloud);
    PointCloud::Ptr cloud_regist (new PointCloud);


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_out) == -1) //* load the file

    {
        PCL_ERROR ("Couldn't read file model \n");
        return (-1);
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_in) == -1) //* load the file


    {
        PCL_ERROR ("Couldn't read file shape \n");
        return (-1);
    }
    

    //-----------------------------------

    Eigen::Matrix4f rotation_matrix;

    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);  //Provide a pointer to the search object. 
    norm_est.setKSearch (30); // Set the number of k nearest neighbors to use for the feature estimation. 

    norm_est.setInputCloud (cloud_in);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*cloud_in, *points_with_normals_src);

    norm_est.setInputCloud (cloud_out);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*cloud_out, *points_with_normals_tgt);

    //   IterativeClosestPoint
    // Align pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
    pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
    icp.setInputSource (points_with_normals_src);
    icp.setInputTarget (points_with_normals_tgt);
    //  icp.setTransformationEpsilon (1e-6);
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    //save cloud for visualization purpose
    points_with_normals_src = reg_result;
    // Estimate
    icp.align (*reg_result);

   PointCloud::Ptr shapePos (new PointCloud);
   transformPointCloud(*cloud_in,*shapePos,icp.getFinalTransformation());
   rotation_matrix = icp.getFinalTransformation();

    double rms = computeCloudRMS(cloud_out,shapePos,std::numeric_limits<double>::max ());
    //vector <double> rms_min = min(rms);

    // double elem1, elem2, elem3, angle123, result;
    //Calculation of rotation after alignment    
        double elem1 = rotation_matrix(0, 0); // r_11
        double elem2 = rotation_matrix(1, 1); // r_22
        double elem3 = rotation_matrix(2, 2); // r_33
        double angle123 = (elem1 + elem2 + elem3 - 1) / 2.0; // Axis–angle representation==(sum of the main diagonal - 1)/2
      double result = acos (angle123) * 180.0 / PI;

    std::cout << "ROTATION: " << result <<":"<< std::endl;
    cout<<"RMSE" << rms<<":"<< endl;
    printf("Tempo:%f",(clock() - tempo) / (double)CLOCKS_PER_SEC);


int sizePoints = 2; // size of points


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("REGISTRO"));

    // Adding both point clouds
    viewer1->setBackgroundColor(255,255,255);

    viewer1->addPointCloud(cloud_out, "cloud_out");
    viewer1->addPointCloud(shapePos, "cloud_in");

    //// Configuring cloud_in
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_in");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");
    // Configuring cloud_out
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 255, 0,"cloud_out");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");

 // Starting visualizer
    // viewer_final->addCoordinateSystem (1.0, "global");
    while(!viewer1->wasStopped())
    {
        viewer1->spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    system("pause");
}
