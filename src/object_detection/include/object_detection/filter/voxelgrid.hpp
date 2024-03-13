#pragma once
#ifndef VOXELGRID_HPP_
#define VOXELGRID_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

template <typename PointPCL>
class voxelgrid
{
public:
    voxelgrid() {outcloud_vol = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);}

    void set_param(const double& x, const double& y, const double& z, const bool& ifall)
    {
        lx = float(x);
        ly = float(y);
        lz = float(z);
        downsample_all = ifall;
        
        success = true;
        return;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {
        if(input->size()>0)
        {
            if(success)
            {
                vol.setInputCloud(input);
                vol.setLeafSize(lx, ly, lz);
                vol.setDownsampleAllData(downsample_all);
                vol.filter(*outcloud_vol);
                pcl::removeNaNFromPointCloud(*outcloud_vol, *outcloud_vol, indices_pth);
                indices_pth.clear();
            } 
        }
        else printf("the input pointcloud for voxelgrid filter is empty, size is %ld\n", input->size());
        return;
    }

public:
    pcl::VoxelGrid<PointPCL> vol;
    std::vector<int> indices_pth;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_vol;
    bool success = false;

private:
    double lx, ly, lz;
    bool downsample_all;
};

#endif