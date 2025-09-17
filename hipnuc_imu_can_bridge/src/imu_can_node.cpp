// imu_can_node.cpp
// ROS node that reads CAN frames, parses them with hipnuc parser, and publishes sensor_msgs/Imu

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>

extern "C" {
#include "hipnuc_imu_can_bridge/hipnuc_can_parser.h" // include path: include/hipnuc_imu_can_bridge/hipnuc_can_parser.h
}
// Helper: convert Euler (radians) to quaternion
static void eulerToQuat(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q) {
    // 使用标准转换（ZYX）
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_can_node");
    ros::NodeHandle nh("~");

    // 参数：can 接口、目标 node id、frame_id
    std::string can_if;
    int node_id;
    std::string frame_id;
    nh.param<std::string>("can_interface", can_if, "can0");
    nh.param<int>("node_id", node_id, 8);
    nh.param<std::string>("frame_id", frame_id, "imu_link");

    ROS_INFO("Starting imu_can_node on interface %s, target node id %d", can_if.c_str(), node_id);

    // 发布者：/imu/data
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);

    // 创建 SocketCAN 套接字（原始 CAN）
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return 1;
    }

    // 获取接口索引
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_if.c_str(), IFNAMSIZ-1);
    ifr.ifr_name[IFNAMSIZ-1] = 0;
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("ioctl SIOCGIFINDEX failed: %s", strerror(errno));
        close(sock);
        return 1;
    }

    // 绑定到接口
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("bind failed: %s", strerror(errno));
        close(sock);
        return 1;
    }

    // 使套接字非阻塞（用于与 ROS 循环配合）
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    ROS_INFO("CAN socket bound to %s (ifindex=%d)", can_if.c_str(), ifr.ifr_ifindex);

    // 循环读取
    ros::Rate loop_rate(1000.0); // 高频循环，但实际发布频率由 incoming 消息决定
    struct can_frame frame;
    hipnuc_can_frame_t hip_frame;
    can_sensor_data_t parsed;
    bool have_quat = false;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = frame_id;

    // 设置默认协方差（用户可根据传感器精度调整）
    // 这里使用 -1 表示未知（ROS 习惯），但 sensor_msgs/Imu 要求非负数组，通常填 0 或估计值
    for (int i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = -1; // 意味着orientation未知（某些消费者识别-1）
        imu_msg.angular_velocity_covariance[i] = -1;
        imu_msg.linear_acceleration_covariance[i] = -1;
    }

    while (ros::ok()) {
        // 使用 select 进行等待（超时 50 ms）
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 50 ms

        int ret = select(sock + 1, &readfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(sock, &readfds)) {
            ssize_t nbytes = read(sock, &frame, sizeof(frame));
            if (nbytes < 0) {
                // 非阻塞情况下 EAGAIN 正常
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // nothing
                } else {
                    ROS_WARN("CAN read error: %s", strerror(errno));
                }
            } else if ((size_t)nbytes >= sizeof(struct can_frame)) {
                // 将 Linux can_frame 转换为 hipnuc_can_frame_t（平台无关）
                hip_frame.can_id = frame.can_id;
                hip_frame.can_dlc = frame.can_dlc;
                // 保证拷贝 8 字节
                memset(hip_frame.data, 0, sizeof(hip_frame.data));
                memcpy(hip_frame.data, frame.data, (frame.can_dlc <= 8 ? frame.can_dlc : 8));

                // 调用解析器
                int msg_type = hipnuc_can_parse_frame(&hip_frame, &parsed, (uint8_t)node_id);

                if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
                    // 根据解析结果更新 imu_msg（注意：不同 msg_type 提供不同字段）
                    ros::Time now = ros::Time::now();
                    imu_msg.header.stamp = now;

                    // 线性加速度
                    if (msg_type == CAN_MSG_ACCEL) {
                        imu_msg.linear_acceleration.x = parsed.accel_x;
                        imu_msg.linear_acceleration.y = parsed.accel_y;
                        imu_msg.linear_acceleration.z = parsed.accel_z;
                        // 如果协方差尚未设置，用户可以在参数中设置真实值；这里保留默认
                    }

                    // 角速度
                    if (msg_type == CAN_MSG_GYRO) {
                        imu_msg.angular_velocity.x = parsed.gyro_x;
                        imu_msg.angular_velocity.y = parsed.gyro_y;
                        imu_msg.angular_velocity.z = parsed.gyro_z;
                    }

                    // 四元数优先作为 orientation
                    if (msg_type == CAN_MSG_QUAT) {
                        geometry_msgs::Quaternion q;
                        q.w = parsed.quat_w;
                        q.x = parsed.quat_x;
                        q.y = parsed.quat_y;
                        q.z = parsed.quat_z;
                        imu_msg.orientation = q;
                        have_quat = true;
                    }

                    // 若收到欧拉角则可转换（仅在没有四元数或需覆盖时使用）
                    if (msg_type == CAN_MSG_EULER || msg_type == CAN_MSG_PITCH_ROLL || msg_type == CAN_MSG_YAW) {
                        // 如果 parser 填充了 roll/pitch/yaw，转换为四元数
                        geometry_msgs::Quaternion q;
                        eulerToQuat(parsed.roll, parsed.pitch, parsed.yaw, q);
                        imu_msg.orientation = q;
                        // note: 可能与 CAN_MSG_QUAT 冲突，按时间戳或消息优先级逻辑选择覆盖策略
                    }

                    // 发布 IMU 消息（在每次解析到有意义字段时发布）
                    imu_pub.publish(imu_msg);
                } // end valid msg
            } // end read success
        } // end select >0

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(sock);
    return 0;
}
