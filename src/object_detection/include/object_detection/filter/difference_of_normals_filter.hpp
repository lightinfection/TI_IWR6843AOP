#pragma once
#ifndef DIFFERENCE_OF_NORMALS_HPP_
#define DIFFERENCE_OF_NORMALS_HPP_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>

struct norm_3d
{
    float norm0_x, norm0_y, norm0_z, norm1_x, norm1_y, norm1_z;
    int index;
};

template <typename PointPCLN, typename PointPCL>
class don_seg
{
public:
    don_seg() 
    {
        normals_small_scale = typename pcl::PointCloud<PointPCLN>::Ptr (new typename pcl::PointCloud<PointPCLN>); 
        normals_large_scale = typename pcl::PointCloud<PointPCLN>::Ptr (new typename pcl::PointCloud<PointPCLN>); 
        doncloud = typename pcl::PointCloud<PointPCLN>::Ptr (new typename pcl::PointCloud<PointPCLN>); 
        outcloud_don = typename pcl::PointCloud<PointPCLN>::Ptr (new typename pcl::PointCloud<PointPCLN>); 

        range_cond = typename pcl::ConditionOr<PointPCLN>::Ptr (new typename pcl::ConditionOr<PointPCLN>);
    }

    void set_param(const double& thr, const double& scale_l, const double& scale_h)
    {
        threshold = thr;
        scale1 = scale_l;
        scale2 = scale_h;
        
        success = true;
        return;
    }

    bool calc_normal(typename pcl::NormalEstimationOMP<PointPCL, PointPCLN>& neomp, const typename pcl::PointCloud<PointPCLN>::Ptr& p, const double& scale)
    {
        if (neomp.getKSearch())
        {
            printf("KSearch is not compatible with DON algorithm, k = %d", neomp.getKSearch());
            return false;
        }
        neomp.setRadiusSearch(scale);
        neomp.compute(*p);
        return true;
    }

    void run(const typename pcl::PointCloud<PointPCL>::Ptr& input)
    {
        if(input->size()>0)
        {
            if(success)
            {
                if(input->isOrganized()) tree.reset(new typename pcl::search::OrganizedNeighbor<PointPCL>);
                else tree.reset (new typename pcl::search::KdTree<PointPCL> (false));
                tree->setInputCloud(input);

                ne.setInputCloud(input);
                ne.setSearchMethod(tree);
                ne.setViewPoint(0,0,0);  // when loaded from octomap;
                if(calc_normal(ne, normals_small_scale, scale1) && calc_normal(ne, normals_large_scale, scale2))
                {
                    if(normals_large_scale->points.size()!=normals_small_scale->points.size()) 
                    {
                        printf("Stop because point numbers dont match. \n");
                        return;
                    }
                    normals.resize(normals_large_scale->points.size());
                    for (size_t i = 0; i < normals_large_scale->points.size(); i++)
                    {
                        normals[i].index = (int)i;
                        normals[i].norm0_x = normals_small_scale->points[i].normal_x;
                        normals[i].norm0_y = normals_small_scale->points[i].normal_y;
                        normals[i].norm0_z = normals_small_scale->points[i].normal_z;
                        normals[i].norm1_x = normals_large_scale->points[i].normal_x;
                        normals[i].norm1_y = normals_large_scale->points[i].normal_y;
                        normals[i].norm1_z = normals_large_scale->points[i].normal_z;
                    }                    
                    don.setInputCloud(input);
                    don.setNormalScaleLarge(normals_large_scale);
                    don.setNormalScaleSmall(normals_small_scale);
                    pcl::copyPointCloud<PointPCL, PointPCLN>(*input, *doncloud);
                    don.computeFeature(*doncloud);
                    range_cond->addComparison(typename pcl::FieldComparison<PointPCLN>::ConstPtr(
                        new typename pcl::FieldComparison<PointPCLN> ("curvature", pcl::ComparisonOps::GT, threshold)));
                    condrem.setCondition(range_cond);
                    condrem.setInputCloud(doncloud);
                    condrem.filter(*outcloud_don);
                }
                else printf("normals calculation goes wrong. \n");
                } 
            }
        else printf("the input pointcloud for difference of normals segmentation is empty, size is %ld\n", input->size());
        return;
    }

public:
    typename pcl::NormalEstimationOMP<PointPCL, PointPCLN> ne;
    typename pcl::ConditionalRemoval<PointPCLN> condrem;
    typename pcl::DifferenceOfNormalsEstimation<PointPCL, PointPCLN, PointPCLN> don;
    
    typename pcl::search::Search<PointPCL>::Ptr tree;
    typename pcl::PointCloud<PointPCLN>::Ptr normals_small_scale, normals_large_scale, doncloud, outcloud_don;
    typename pcl::ConditionOr<PointPCLN>::Ptr range_cond;
    
    bool success = false;
    std::vector<norm_3d> normals;

private:
    double threshold, scale1, scale2;
};

#endif