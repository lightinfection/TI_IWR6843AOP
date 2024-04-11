#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>
#include <pcl/io/pcd_io.h>
#include "object_detection/common/param.h"
#include "object_detection/filter/difference_of_normals_filter.hpp"
#include "object_detection/filter/statistical_outlier_removal.hpp"
#include "object_detection/filter/radius_outlier_removal.hpp"
#include "object_detection/filter/voxelgrid.hpp"
#include "object_detection/cluster/dbscan.hpp"

enum filter_type
{
  StatisticalOutlierRemoval = 1,
  RadiusOutlierRemoval = 2,
  DBSCAN = 3,
  VoxelGrid = 4,
};

class minimalsubscriber : public rclcpp::Node
{
public:
  minimalsubscriber()
  : Node("ti_mmwave")
  {
    get_param();
    if (don_param.if_channel && filter_param.name!=filter_type::DBSCAN) don.set_param(don_param.eps, don_param.lower_limit, don_param.upper_limit);
    if (don_param.if_channel && filter_param.name==filter_type::DBSCAN) don_rgb.set_param(don_param.eps, don_param.lower_limit, don_param.upper_limit);
    oricloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    bool ready = getPointcloud(map_, oricloud);
    if (ready)
    {
      sensor_qos.keep_last(10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&minimalsubscriber::topic_callback, this));
      publisher0_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_raw", sensor_qos);
      pcl::toROSMsg(*oricloud, output_raw);
      if(don.success || don_rgb.success)
      {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_DON_filt_out", sensor_qos);
        if(sor.success || ror.success || dbs.success || vol.success)
        {
          publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_filtered", sensor_qos);
          switch (filter_param.name)
          {
            case filter_type::StatisticalOutlierRemoval:
              sor.run(oricloud);
              pcl::toROSMsg(*sor.outcloud_sat, output_filter);
              don.run(sor.outcloud_sat);
              break;
            case filter_type::RadiusOutlierRemoval:
              ror.run(oricloud);
              pcl::toROSMsg(*ror.outcloud_rad, output_filter);
              don.run(ror.outcloud_rad);
              break;
            case filter_type::DBSCAN:
              dbs.run(oricloud);
              pcl::toROSMsg(*dbs.outcloud_dbscan, output_filter);
              don_rgb.run(dbs.outcloud_dbscan);
              break;
            case filter_type::VoxelGrid:
              vol.run(oricloud);
              pcl::toROSMsg(*vol.outcloud_vol, output_filter);
              don.run(vol.outcloud_vol);
              break;
          }
        }
        else don.run(oricloud);
        if(dbs.success) pcl::toROSMsg(*don_rgb.outcloud_don, output_don);
        else pcl::toROSMsg(*don.outcloud_don, output_don);
      } 
      else RCLCPP_ERROR(this->get_logger(), "parameters of different of normals setting went wrong"); 
    }
  }

private:
  void topic_callback()
  {
    output_raw.header.stamp = rclcpp::Clock().now();
    publisher0_->publish(output_raw);
    if(sor.success || ror.success || dbs.success || vol.success)
    {
      output_filter.header.stamp = rclcpp::Clock().now();
      publisher1_->publish(output_filter);
    }
    if (don.success || don_rgb.success) 
    {
      output_don.header.stamp = rclcpp::Clock().now();
      publisher_->publish(output_don);
    }
    return;
  }
  
  void get_param()
  {
    map_ = this->declare_parameter("map", "map.pcd");
    organized_ = this->declare_parameter("isOrganized", false);
    filter_param.name = this->declare_parameter("filter", 0);
    switch (filter_param.name)
    {
      case 0:
        break;
      case filter_type::StatisticalOutlierRemoval:
        set_params(filter_param, "stddev", "mean_K");
        sor.set_param(filter_param.eps, filter_param.num, false);
        break;
      case filter_type::RadiusOutlierRemoval:
        set_params(filter_param, "radius", "mean_K");
        ror.set_param(filter_param.eps, filter_param.num, false);
        break;
      case filter_type::DBSCAN:
        set_params(filter_param, "radius", "mean_K");
        dbs.set_params(filter_param.eps, filter_param.num, false, "", "");
        break;
      case filter_type::VoxelGrid:
        set_params(filter_param, "lx", "", "ly", "lz");
        vol.set_param(filter_param.eps, filter_param.lower_limit, filter_param.upper_limit, false);
        break;
    }

    don_param.if_channel = this->declare_parameter("DON", true);
    if (don_param.if_channel) set_params(don_param, "theshold", "", "lower_limit", "upper_limit");
  }

  void set_params(param& ch, const std::string& eps_, std::string num_ = "mean_K", std::string supplement0_ = "0", std::string supplement1_ = "1")
  {
    ch.eps = this->declare_parameter(eps_, 0.1);
    if(num_=="mean_K") ch.num = this->declare_parameter(num_, 10);
    if (supplement0_!="0") ch.lower_limit = this->declare_parameter(supplement0_, 0.1);
    if (supplement1_!="1") ch.upper_limit = this->declare_parameter(supplement1_, 0.1);
  }
  
  bool getPointcloud(const std::string &map_type, pcl::PointCloud<pcl::PointXYZ>::Ptr& oricloud_)
  {
    oricloud_->clear();
    std::string ss = map_type;
    int del = ss.find(".");
    while (del!=-1)
    {
      ss.erase(ss.begin(), ss.begin()+del+1);
      del = ss.find(".");
    }

    // RCLCPP_INFO(this->get_logger(), "The input map is : %s, and type is %s", map_type.c_str(), ss.c_str());
    if (ss != "bt" && ss != "ot" && ss != "pcd")
    {
      RCLCPP_ERROR(this->get_logger(), "The input map is neither a .bt octomap nor a .pcd pointcloud map.");
      return false;
    }

    if (ss == "bt" || ss == "ot")
    {
      octomap::OcTree* tree = new octomap::OcTree(0.1);
      if (!tree->readBinary(map_type))
      {
        OCTOMAP_ERROR("Could not open file, exiting.\n");
        exit(1);
      }
      unsigned int maxDepth = tree->getTreeDepth();
      // expand collapsed occupied nodes until all occupied leaves are at maximum depth
      std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
      do {
        collapsed_occ_nodes.clear();
        for (octomap::OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
        {
          if(tree->isNodeOccupied(*it) && it.getDepth() < maxDepth)
          {
            collapsed_occ_nodes.push_back(&(*it));
          }
        }
        for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
        {
          tree->expandNode(*it);
        }
      } while(collapsed_occ_nodes.size() > 0);

      std::vector<octomap::point3d> pcl_;
      for (octomap::OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
      {
        if(tree->isNodeOccupied(*it))
        {
          pcl_.push_back(it.getCoordinate());
        }
      }
      delete tree;
      
      oricloud_->width = 1;
      oricloud_->height = pcl_.size();
      oricloud_->is_dense = false;
      oricloud_->resize(oricloud->width*oricloud->height);
      oricloud_->header.frame_id = std::string("/map");
      for (size_t i = 0; i < pcl_.size(); i++)
      {
        oricloud_->points[i].x = pcl_[i].x();
        oricloud_->points[i].y = pcl_[i].y();
        oricloud_->points[i].z = pcl_[i].z();
      } 
      if (!organized_)
      {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*oricloud_, *oricloud_, indices);
      }
      return true;
    }

    if (ss == "pcd")
    {
      if(!pcl::io::loadPCDFile(map_type, *oricloud_))
      {
        PCL_ERROR("Could not open file, exiting.\n");
        exit(1);
      }
    }
      if (!organized_)
      {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*oricloud_, *oricloud_, indices);
      }
      return true;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, publisher0_, publisher1_;
  rclcpp::SensorDataQoS sensor_qos;
  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr oricloud;
  sensor_msgs::msg::PointCloud2 output_don, output_raw, output_filter;
  stat_outlier<pcl::PointXYZ> sor;
  radius_outlier<pcl::PointXYZ> ror;
  voxelgrid<pcl::PointXYZ> vol;
  dbscan<pcl::PointXYZ, pcl::PointXYZRGB> dbs;
  don_seg<pcl::PointNormal, pcl::PointXYZ> don;
  don_seg<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> don_rgb;
  
  bool organized_;
  std::string map_;
  param filter_param, don_param;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<minimalsubscriber>()); 
  rclcpp::shutdown();
}