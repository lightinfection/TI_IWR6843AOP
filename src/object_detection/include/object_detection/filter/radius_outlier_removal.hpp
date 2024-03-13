#pragma once
#ifndef RADIUS_OUTLIER_REMOVAL_HPP_
#define RADIUS_OUTLIER_REMOVAL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>

template <typename PointPCL>
class radius_outlier
{
public:
    radius_outlier() {outcloud_rad = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);}

    void set_param(const float& r, const int& m, const bool& ifreverse)
    {
        radius = r;
        min_points = m;
        reverse = ifreverse;
        
        success = true;
        return;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {
        if(input->size()>0)
        {
            if(success)
            {
                rad.setInputCloud(input);
                rad.setRadiusSearch(radius);
                rad.setMinNeighborsInRadius(min_points);
                // rad.setKeepOrganized(true);
                rad.setNegative(reverse);
                rad.filter(*outcloud_rad);
                pcl::removeNaNFromPointCloud(*outcloud_rad, *outcloud_rad, indices_pth);
                indices_pth.clear();
            } 
        }
        else printf("the input pointcloud for radius outlier removal is empty, size is %ld\n", input->size());
        return;
    }

public:
    pcl::RadiusOutlierRemoval<PointPCL> rad;
    std::vector<int> indices_pth;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_rad;
    bool success = false;

private:
    bool reverse;
    float radius;
    int min_points;
};

#endif