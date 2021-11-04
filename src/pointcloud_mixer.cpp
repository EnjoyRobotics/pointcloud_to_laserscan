/*
 *  Two YDLIDARs Mixer V2.0
 *  Enjoy Robotics
 */

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class cloud_Mixer : public rclcpp::Node
{
    public:
    cloud_Mixer()
    : Node("pointcloud_mixer")
        {
            lidar_info_sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "/scanner/cloud1", rclcpp::SensorDataQoS(), std::bind(&cloud_Mixer::pointcloudCb1, this, _1));
            
            lidar_info_sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "/scanner/cloud2", rclcpp::SensorDataQoS(), std::bind(&cloud_Mixer::pointcloudCb2, this, _1));

            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

            timer_ = this->create_wall_timer(
                    20ms, std::bind(&cloud_Mixer::pointcloudCb, this));

        }
        sensor_msgs::msg::PointCloud2 cloud_msg;

        std::vector<uint8_t> cloud1;
        std::vector<uint8_t> cloud2;
        std::vector<uint8_t> cloud;
        uint32_t width1;
        uint32_t width2;
        int size;

    private:
        void pointcloudCb1 (const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
        {
            cloud1 = point_cloud2_msgs->data;
            width1 = point_cloud2_msgs->width;
        }
        void pointcloudCb2 (const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
        {
            
            cloud2 = point_cloud2_msgs->data;
            width2 = point_cloud2_msgs->width;
            
            cloud_msg.header.stamp.sec = point_cloud2_msgs -> header.stamp.sec;
            cloud_msg.header.stamp.nanosec = point_cloud2_msgs->header.stamp.nanosec;
            cloud_msg.header.frame_id = point_cloud2_msgs->header.frame_id ;
            cloud_msg.fields = point_cloud2_msgs->fields;
            cloud_msg.height = point_cloud2_msgs->height;
            cloud_msg.is_bigendian = point_cloud2_msgs->is_bigendian;
            cloud_msg.width = width1 + width2;
            cloud_msg.point_step = point_cloud2_msgs -> point_step;
            cloud_msg.row_step = point_cloud2_msgs->row_step;
            cloud_msg.is_dense = point_cloud2_msgs->is_dense;

            cloud.erase(cloud.begin(),cloud.end());

            cloud.reserve(cloud.size() + cloud1.size());
            cloud.insert(cloud.end(), cloud1.begin(), cloud1.end());
            cloud.reserve(cloud.size() + cloud2.size());
            cloud.insert(cloud.end(), cloud2.begin(), cloud2.end());

            cloud_msg.data = cloud;
        }
        void pointcloudCb()
        {
            pointcloud_pub_->publish(std::move(cloud_msg));
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_info_sub1_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_info_sub2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cloud_Mixer>());
  rclcpp::shutdown();
  return 0;
}
