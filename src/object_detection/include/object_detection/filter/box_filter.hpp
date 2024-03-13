#pragma once
#ifndef BOX_FILTER_HPP_
#define BOX_FILTER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

template <typename PointPCL>
class box_filter
{
public:
    box_filter() {outcloud_box = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);}

    void set_border(const double& x0, const double& y0, const double& z0, const double& x1, const double& y1, const double& z1, const bool& ifreverse)
    {
        x_min = x0;
        y_min = y0;
        z_min = z0;
        x_max = x1;
        y_max = y1;
        z_max = z1;
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
                // filters
                // 1st layer: Box-shape-filter X: -0.3~0.3; Y: <0.5; Z: 0.3~0.3
                box.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
                box.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
                box.setNegative(reverse);
                box.setInputCloud(input);
                box.filter(*outcloud_box);
                pcl::removeNaNFromPointCloud(*outcloud_box, *outcloud_box, indices_pth);
                indices_pth.clear();
            } 
        }
        else printf("the input pointcloud is empty, size is %ld\n", input->size());
        return;
    }

public:
    pcl::CropBox<PointPCL> box;
    std::vector<int> indices_pth;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_box;
    bool success = false;

private:
    bool reverse;
    double x_min, y_min, z_min, x_max, y_max, z_max;
};

#endif