#pragma once
#ifndef ICP_HPP_
#define ICP_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>

template <typename PointPCL>
class icp3d
{
public:
    icp3d()
    {
        outcloud_icp = typename pcl::PointCloud<PointPCL>::Ptr(new typename pcl::PointCloud<PointPCL>);
    }

    void set_params(const int &itr, const float &dist, const bool& RANSAC_)
    {
        max_itr = itr;
        max_dist = dist;
        if_RANSAC = RANSAC_;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {

    }

public:
    IterativeClosestPoint<PointPCL> icp;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_icp;

private:
    int max_itr;
    bool if_RANSAC;
    float max_dist;
}

#endif