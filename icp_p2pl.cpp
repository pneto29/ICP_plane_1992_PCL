#include <iostream>
using namespace std;

#include <algorithm>
#include <string> 
#include <vector>
using std::vector;
#define PI 3.14159265

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <boost/thread/thread.hpp>

#include <pcl/console/time.h> 
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <boost/thread/thread.hpp>

#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
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
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr cloud_in (new PointCloud);
    PointCloud::Ptr cloud_out (new PointCloud);
    PointCloud::Ptr cloud_icp (new PointCloud);
    PointCloud::Ptr cloud_regist (new PointCloud);

    //PARAMETRO - NUVEM
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
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);
    //std::cout << "k: " << setKSearch << std::endl;

    norm_est.setInputCloud (cloud_in);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*cloud_in, *points_with_normals_src);

    norm_est.setInputCloud (cloud_out);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*cloud_out, *points_with_normals_tgt);

    //   IterativeClosestPoint

    // Align     pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;

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

    double elem1, elem2, elem3, angle123,result;
    elem1 = rotation_matrix(0,0);

    elem2 = rotation_matrix(1,1);
    elem3= rotation_matrix(2,2);
    angle123= (elem1+elem2+elem3-1)/2;
    result = acos (angle123) * 180.0 / PI;

    double trans1, trans2, trans3, result2, trans12, trans22, trans32;
    trans1 = rotation_matrix(0,3);
    trans2 = rotation_matrix(1,3);
    trans3 = rotation_matrix(2,3);
    trans12= trans1*trans1;
    trans22= trans2*trans2;
    trans32= trans3*trans3;
    result2=sqrt(trans12+trans22+trans32);

    std::cout << "ROTAÇÃO: " << result <<":"<< std::endl;
    std::cout << "TRANSLAÇÃO: " << ":"<<result2 << std::endl;
    cout<<"RMSE" << rms<<":"<< endl;
    printf("Tempo:%f",(clock() - tempo) / (double)CLOCKS_PER_SEC);
}
