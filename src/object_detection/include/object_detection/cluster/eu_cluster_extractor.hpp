#pragma once
#ifndef EU_CLUSTER_EXTRACTOR_HPP_
#define EU_CLUSTER_EXTRACTOR_HPP_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

template <typename PointPCL>
class EuclideanClusterExtraction
{
public:
    EuclideanClusterExtraction() 
    {
        outcloud_ec = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);
    }

    void set_param(const int& min, const int& max, const double& tol)
    {
        min_size = min;
        max_size = max;
        tolerance = tol;
        
        success = true;
        return;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {
        if(input->size()>0)
        {
            if(success)
            {
                if (input->isOrganized()) tree.reset(new pcl::search::OrganizedNeighbor<PointPCL>);
                else tree.reset (new pcl::search::KdTree<PointPCL> (false));
                
                ec.setInputCloud(input);
                ec.setMinClusterSize(min_size);
                ec.setMaxClusterSize(max_size);
                ec.setClusterTolerance(tolerance);
                ec.setSearchMethod(tree);
                ec.extract(clusterID);
            }
        }
        else printf("the input pointcloud for Euclidean Cluster Extraction is empty, size is %ld\n", input->size());
        return;
    }

public:
    typename pcl::EuclideanClusterExtraction<PointPCL> ec;
    std::vector<pcl::PointIndices> clusterID;
    pcl::search::Search<PointPCL>::Ptr tree;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_ec;
    bool success = false;

private:
    int min_size, max_size;
    double tolerance;
};

#endif