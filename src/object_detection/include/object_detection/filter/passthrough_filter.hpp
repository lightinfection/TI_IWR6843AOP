#pragma once
#ifndef PASSTHROUGH_FILTER_HPP_
#define PASSTHROUGH_FILTER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

template <typename PointPCL>
class passthrough_filter
{
public:

typedef struct channel_
{
    double low, up;
    bool neg;
    bool is_set = false;
    typename pcl::PassThrough<PointPCL> pth;
} channel;

    passthrough_filter()
    {
        outcloud_pth = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);
        tmp = typename pcl::PointCloud<PointPCL>::Ptr (new typename pcl::PointCloud<PointPCL>);
        x_c = new channel;
        y_c = new channel;
        z_c = new channel;
        i_c = new channel;
    }

    ~passthrough_filter() 
    {
        if(x_c) 
        {
            delete x_c;
            x_c = NULL;
        }
        if(y_c)
        {
            delete y_c;  
            y_c = NULL;
        } 
        if(z_c)
        {
            delete z_c;
            z_c = NULL;
        }
        if(i_c)
        {
            delete i_c;
            i_c = NULL;
        }
    }

    void set_border(channel* ch, const double& lower, const double& upper, const bool& ifreverse)
    {
        ch->low = lower;
        ch->up = upper;
        ch->neg = ifreverse;
        ch->is_set = true;

        success = true;
        return;
    }

    void set_filter(pcl::PassThrough<PointPCL>& pth, const typename pcl::PointCloud<PointPCL>::Ptr& pc, const channel* ch, const std::string& name)
    {
        pth.setInputCloud(pc);
        pth.setFilterFieldName(name);
        pth.setFilterLimits(ch->low, ch->up);
        pth.setNegative(ch->neg);
        pth.filter(*pc);
    }

    void setall(const bool& ifx, const double& x0, const double& x1, const bool& ifreverse_x,
                const bool& ify, const double& y0, const double& y1, const bool& ifreverse_y,
                const bool& ifz, const double& z0, const double& z1, const bool& ifreverse_z,
                const bool& ifi, const double& i0, const double& i1, const bool& ifreverse_i)
    {
        if(ifx) set_border(x_c, x0, x1, ifreverse_x);
        else delete x_c;
        if(ify) set_border(y_c, y0, y1, ifreverse_y);
        else delete y_c;
        if(ifz) set_border(z_c, z0, z1, ifreverse_z);
        else delete z_c;
        if(ifi) set_border(i_c, i0, i1, ifreverse_i);
        else delete i_c;

        return;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {
        if(input->size()>0)
        {
            if(success)
            {
                pcl::copyPointCloud(*input, *tmp);
                if(x_c->is_set) set_filter(x_c->pth, tmp, x_c, "x");
                if(y_c->is_set) set_filter(y_c->pth, tmp, y_c, "y");
                if(z_c->is_set) set_filter(z_c->pth, tmp, z_c, "z");
                if(i_c->is_set) set_filter(i_c->pth, tmp, i_c, "intensity");
                pcl::removeNaNFromPointCloud(*tmp, *outcloud_pth, indices_pth);
                indices_pth.clear();
            }
        }
        else printf("the input pointcloud for passthrough filter is empty, size is %ld\n", input->size());
        return;
    }

public:
    std::vector<int> indices_pth;
    typename pcl::PointCloud<PointPCL>::Ptr outcloud_pth;
    typename pcl::PointCloud<PointPCL>::Ptr tmp;
    bool success = false;

private:
    bool reverse, if_x, if_y, if_z, if_i;
    channel* x_c;
    channel* y_c;
    channel* z_c;
    channel* i_c;
};
#endif