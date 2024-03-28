#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "object_detection/cluster/dbscan.hpp"

class minimalsubscriber : public rclcpp::Node
{
public:
  minimalsubscriber()
  : Node("dbscan_cluster")
  {
    get_param();
    db.set_params(r, num, bb, frame, node_name);
    sensor_qos.keep_last(10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl", sensor_qos, std::bind(&minimalsubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed! \n");
    if(db.success) 
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dbscan_output", sensor_qos);
      if (bb) publisher_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("dbscan_box", 10);
    }
    else RCLCPP_ERROR(this->get_logger(), "parameters setting went wrong\n");
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
  {
    db.run(input);
    pcl::toROSMsg(*db.outcloud_dbscan, output);
    publisher_->publish(output);
    if (bb) publisher_marker->publish(*db.bboxes);

    return;
  }

  void get_param()
  {
    r = this->declare_parameter("eps", 0.8);
    num = this->declare_parameter("minimum_points", 15);
    bb = this->declare_parameter("bounding_boxes", true);
    frame = this->declare_parameter("target_frame", "map");
    node_name = this->declare_parameter("name_box", "dbscan_cluster");
    RCLCPP_INFO(this->get_logger(), "set done: %.2f, %d, %d, %s, %s \n", r, num, bb, frame.c_str(), node_name.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker;
  rclcpp::SensorDataQoS sensor_qos;
  
  dbscan<pcl::PointXYZ, pcl::PointXYZRGB> db;
  sensor_msgs::msg::PointCloud2 output;

  double r;
  int num;
  bool bb;
  std::string frame, node_name;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<minimalsubscriber>());
  rclcpp::shutdown();
}
