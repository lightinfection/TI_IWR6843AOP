#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "object_detection/common/PointTI.h"
#include "object_detection/common/param.h"
#include "object_detection/filter/passthrough_filter.hpp"
#include "object_detection/filter/statistical_outlier_removal.hpp"

class minimalsubscriber : public rclcpp::Node
{
public:
  minimalsubscriber()
  : Node("ti_mmwave")
  {
    get_param();
    pth.setall(pth_param.x.if_channel, pth_param.x.lower_limit, pth_param.x.upper_limit, pth_param.x.if_reverse,
               pth_param.y.if_channel, pth_param.y.lower_limit, pth_param.y.upper_limit, pth_param.y.if_reverse,
               pth_param.z.if_channel, pth_param.z.lower_limit, pth_param.z.upper_limit, pth_param.z.if_reverse,
               pth_param.i.if_channel, pth_param.i.lower_limit, pth_param.i.upper_limit, pth_param.i.if_reverse);
    if (sor_param.if_channel) sor.set_param(sor_param.lower_limit, sor_param.upper_limit, sor_param.if_reverse);
    sensor_qos.keep_last(10);
    oricloud = pcl::PointCloud<PointTI>::Ptr (new pcl::PointCloud<PointTI>);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl", sensor_qos, std::bind(&minimalsubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed! \n");
    if (!pth.success && !sor.success) {RCLCPP_ERROR(this->get_logger(), "filter parameters setting went wrong\n");}
    if (pth.success) publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_PTH_filt_out_0", sensor_qos);
    if (sor.success) publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_SOR_filt_out_0", sensor_qos);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
  {
    pcl::fromROSMsg(*input, *oricloud);
    if(pth.success)
    {
      pth.run(oricloud);
      pcl::toROSMsg(*pth.outcloud_pth, output);
      publisher_->publish(output);
      if (sor.success)
      {
        sor.run(pth.outcloud_pth);
        pcl::toROSMsg(*sor.outcloud_sat, output_1);
        publisher1_->publish(output_1);
      }
    }
    else
    {
      if (sor.success)
      {
        sor.run(oricloud);
        pcl::toROSMsg(*sor.outcloud_sat, output_1);
        publisher1_->publish(output_1);
      }
    }
    return;
  }

  void get_param()
  {
    this->declare_parameter("x", pth_param.x.if_channel);
    pth_param.x.if_channel = this->get_parameter("x").as_bool();
    this->declare_parameter("y", pth_param.y.if_channel);
    pth_param.y.if_channel = this->get_parameter("y").as_bool();
    this->declare_parameter("z", pth_param.z.if_channel);
    pth_param.z.if_channel = this->get_parameter("z").as_bool();
    this->declare_parameter("i", pth_param.i.if_channel);
    pth_param.i.if_channel = this->get_parameter("i").as_bool();
    this->declare_parameter("SOR", sor_param.if_channel);
    sor_param.if_channel = this->get_parameter("SOR").as_bool();
    if (pth_param.x.if_channel) set_params(pth_param.x, "reverse_x", "x_min", "x_max");
    if (pth_param.y.if_channel) set_params(pth_param.y, "reverse_y", "y_min", "y_max");
    if (pth_param.z.if_channel) set_params(pth_param.z, "reverse_z", "z_min", "z_max");
    if (pth_param.i.if_channel) set_params(pth_param.i, "reverse_i", "i_min", "i_max");
    if (sor_param.if_channel) set_params(sor_param, "reverse_sor", "stddev", "mean_K");
  }

  void set_params(param& ch, const std::string& reverse_, const std::string& min_, const std::string& max_)
  {
    ch.if_reverse = this->declare_parameter(reverse_, false);
    ch.lower_limit = this->declare_parameter(min_, 0.1);
    ch.upper_limit = this->declare_parameter(max_, 1.1);
    RCLCPP_INFO(this->get_logger(), "set done: %.2f, %.2f, %d \n", ch.lower_limit, ch.upper_limit, ch.if_reverse);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, publisher1_;
  rclcpp::SensorDataQoS sensor_qos;

  passthrough_filter<PointTI> pth;
  stat_outlier<PointTI> sor;
  pcl::PointCloud<PointTI>::Ptr oricloud;
  sensor_msgs::msg::PointCloud2 output, output_1;

  config pth_param;
  param sor_param;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<minimalsubscriber>());
  rclcpp::shutdown();
}
