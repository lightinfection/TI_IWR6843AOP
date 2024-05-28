#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include "object_detection/common/param.h"
#include "object_detection/filter/statistical_outlier_removal.hpp"
#include "pcl/sample_consensus/ransac.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

class minimalsubscriber : public rclcpp::Node
{
public:
  minimalsubscriber()
  : Node("ti_mmwave")
  {
    oricloud0 = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    oricloud1 = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    aligned = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    std::string map_0 = "/home/lscm/Desktop/TI_IWR6843AOP/src/object_detection/map/1713338848546331757.pcd";
    std::string map_1 = "/home/lscm/Desktop/TI_IWR6843AOP/src/object_detection/map/1713338868543509855.pcd";
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    bool ready = getPointcloud(map_0, oricloud0) && getPointcloud(map_1, oricloud1);
    icp.setInputSource(oricloud0);    // icp.setMaxCorrespondenceDistance(1); //设置对应点容忍最大距离
    icp.setInputTarget(oricloud1);
    icp.setMaximumIterations(50);    //设置最大迭代次数
    icp.setRANSACIterations(1);    //不

    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (oricloud0));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();
    std::vector <int> inliers;
    ransac.getInliers(inliers);
    RCLCPP_INFO(this->get_logger(), "max itr: %d", ransac.getMaxIterations());
    pcl::copyPointCloud(*oricloud0, inliers, *oricloud1);
    if (ready)
    {
      sensor_qos.keep_last(10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&minimalsubscriber::topic_callback, this));
      publisher0_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_0", sensor_qos);
      publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_1", sensor_qos);
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_", sensor_qos);
      pcl::toROSMsg(*oricloud0, output_0);
      pcl::toROSMsg(*oricloud1, output_1);
      // icp.align(*aligned);
      // pcl::toROSMsg(*aligned, output_);
      // RCLCPP_INFO(this->get_logger(), "is converged? : %d", icp.hasConverged());
      // RCLCPP_INFO(this->get_logger(), "score : %.4f", icp.getFitnessScore());
      // RCLCPP_INFO(this->get_logger(), "1nd: %d, 2nd: %d, fusion: %d", oricloud0->size(), oricloud1->size(), aligned->size());
    }
  }

private:
  void topic_callback()
  {
    output_0.header.stamp = rclcpp::Clock().now();
    output_1.header.stamp = rclcpp::Clock().now();
    // output_.header.stamp = rclcpp::Clock().now();
    publisher0_->publish(output_0);
    publisher1_->publish(output_1);
    // publisher_->publish(output_);
    return;
  }
  
  // void get_param()
  // {
  // }

  // void set_params()
  // {
  // }
  
  bool getPointcloud(const std::string &map_type, pcl::PointCloud<pcl::PointXYZI>::Ptr& oricloud_)
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
      oricloud_->resize(oricloud_->width*oricloud_->height);
      oricloud_->header.frame_id = std::string("/map");
      for (size_t i = 0; i < pcl_.size(); i++)
      {
        oricloud_->points[i].x = pcl_[i].x();
        oricloud_->points[i].y = pcl_[i].y();
        oricloud_->points[i].z = pcl_[i].z();
      } 
      // if (!organized_)
      // {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*oricloud_, *oricloud_, indices);
      // }
      return true;
    }

    if (ss == "pcd")
    {
      if(pcl::io::loadPCDFile(map_type, *oricloud_)==-1)
      {
        PCL_ERROR("Could not open file, exiting.\n");
        exit(1);
      }
      oricloud_->header.frame_id = std::string("/map");
      // if (!organized_)
      // {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*oricloud_, *oricloud_, indices);
      // }
      return true;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher0_, publisher1_, publisher_;
  rclcpp::SensorDataQoS sensor_qos;
  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr oricloud0, oricloud1, aligned;
  sensor_msgs::msg::PointCloud2 output_0, output_1, output_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<minimalsubscriber>()); 
  rclcpp::shutdown();
}