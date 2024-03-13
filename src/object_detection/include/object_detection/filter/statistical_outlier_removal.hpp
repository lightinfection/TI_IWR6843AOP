#pragma once
#ifndef STATISTICAL_OUTLIER_REMOVAL_HPP_
#define STATISTICAL_OUTLIER_REMOVAL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

template <typename PointPCL>
class stat_outlier
{
public:
    stat_outlier() {outcloud_sat = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);}

    void set_param(const float& dev, const int& mk, const bool& ifreverse)
    {
        stddev = dev;
        meank = mk;
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
                sat.setInputCloud(input);
                sat.setMeanK(meank);
                sat.setStddevMulThresh(stddev);
                sat.setNegative(reverse);
                sat.filter(*outcloud_sat);
                pcl::removeNaNFromPointCloud(*outcloud_sat, *outcloud_sat, indices_pth);
                indices_pth.clear();
            } 
        }
        else printf("the input pointcloud for statistical outlier removal is empty, size is %ld\n", input->size());
        return;
    }

public:
    pcl::StatisticalOutlierRemoval<PointPCL> sat;
    std::vector<int> indices_pth;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_sat;
    bool success = false;

private:
    bool reverse;
    float stddev;
    int meank;
};

#endif