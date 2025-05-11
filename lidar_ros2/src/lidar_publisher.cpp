#include "base/lidar.h"
#include "base/hchead.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <thread>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_publisher_node");
    
    // 设置较大的队列大小
    auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);

    int com_id = 0;
#ifdef _WIN32
    com_id = 3;
#else
    com_id = 0;
#endif
    if (argc >= 2)
    {
        com_id = atoi(argv[1]);
    }
    char buff[32];
#ifdef _WIN32
    sprintf(buff, "//./com%d", com_id);
#else
    sprintf(buff, "/dev/ttyUSB%d", com_id);
#endif

    Dev device;
    int rs = device.Initialize(buff, 115200, true);
    if (rs != 0)
    {
        printf("Initialize failed.\n");
        return 0;
    }

    // 增加发布频率
    rclcpp::Rate loop_rate(5);  // 发布频率 20Hz
    while (rclcpp::ok())
    {
        std::list<node_info> dataList;
        device.GetScanData(dataList, false);
        int errcode = device.GetLastErrCode();
        if (errcode != LIDAR_SUCCESS)
        {
            printf("errcode:%d\n", errcode);
        }
        if (dataList.empty())
        {
            rclcpp::spin_some(node);
            loop_rate.sleep();
            continue;
        }
        // 将数据转换为 vector 并排序（按角度升序，角度 = angle_q6_checkbit/64.0）
        std::vector<node_info> points(dataList.begin(), dataList.end());
        std::sort(points.begin(), points.end(), [](const node_info &a, const node_info &b){
            return a.angle_q6_checkbit < b.angle_q6_checkbit;
        });
        
        // 构造 LaserScan 消息
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = node->now();
        scan_msg.header.frame_id = "laser_frame";

        // 计算最小与最大角度（以弧度计）
        double angle_min_deg = points.front().angle_q6_checkbit / 64.0;
        double angle_max_deg = points.back().angle_q6_checkbit / 64.0;
        scan_msg.angle_min = angle_min_deg * M_PI / 180.0;
        scan_msg.angle_max = angle_max_deg * M_PI / 180.0;
        int num_points = points.size();
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / num_points;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 100.0;
        scan_msg.ranges.resize(num_points);
        scan_msg.intensities.resize(num_points);

        // 填充激光雷达数据，距离 = distance_q2/4.0
        float front_dist = -1, right_dist = -1, back_dist = -1, left_dist = -1;
        for (size_t i = 0; i < points.size(); i++) {
            scan_msg.ranges[i] = points[i].distance_q2 / 4.0;
            scan_msg.intensities[i] = points[i].syn_quality;
            
            // 获取四个方向的距离
            float angle = points[i].angle_q6_checkbit / 64.0;
            if (angle >= 355 || angle <= 5) front_dist = scan_msg.ranges[i];  // 前方 0°
            if (angle >= 85 && angle <= 95) right_dist = scan_msg.ranges[i];  // 右方 90°
            if (angle >= 175 && angle <= 185) back_dist = scan_msg.ranges[i];  // 后方 180° 
            if (angle >= 265 && angle <= 275) left_dist = scan_msg.ranges[i];  // 左方 270°
        }

        // 打印四个方向的距离信息(单位:米)
        printf("\033[2J\033[H");  // 清屏并将光标移到开始位置
        printf("距离信息 (单位:米):\n");
        printf("     前方: %.2f\n", front_dist);
        printf("左侧: %.2f   右侧: %.2f\n", left_dist, right_dist);
        printf("     后方: %.2f\n", back_dist);

        publisher->publish(scan_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    device.Uninit();
    rclcpp::shutdown();
    return 0;
}
