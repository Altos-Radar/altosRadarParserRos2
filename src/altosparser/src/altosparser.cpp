#include "altos-pc-v2.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <array>
#include "ament_index_cpp/get_package_share_directory.hpp"
using namespace std;
#define vrMax 60
#define vrMin -60
#define vStep 0.1
#define errThr 3
#define PI 3.1415926
#define FLIPELELVATION -1
#define INSTALLHEIGHT 1.85

struct RadarUnit
{
    std::string topicName;
    std::array<double, 6> installParam;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud;

    vector<POINTCLOUD> pointCloudVec;

    uint32_t cntPointCloud = 0;
};

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6f) * snr / 5.0e6f / rcsBuf[ind];

    return rcs;
}

int socketGen(string groupIp, int groupPort, string localIp, int uniPort, bool uniFlag)
{
    struct sockaddr_in addr;

    struct ip_mreq req;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return -1;
    }
    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    memset(&addr, 0, sizeof(addr));
    if(uniFlag)
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(uniPort);
        addr.sin_addr.s_addr = inet_addr(localIp.c_str());
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return -1;
        }
    }else
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(groupPort);
        addr.sin_addr.s_addr = INADDR_ANY;
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return -1;
        }

        req.imr_multiaddr.s_addr = inet_addr(groupIp.c_str());
        req.imr_interface.s_addr = inet_addr(localIp.c_str());
        ;
        ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
        if (ret < 0) {
            perror("setsockopt");
            return -1;
        }
    }

    return sockfd;
}

float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float yaw)
{
    int ind = 0;
    float vr = 0;

    for (size_t i = 0; i < pointCloudVec.size(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            if (abs(pointCloudVec[i].point[j].range) > 0)
            {
                vr = pointCloudVec[i].point[j].doppler /
                     cos(pointCloudVec[i].point[j].azi + yaw);
                if (vr > vrMax || vr < vrMin || isnan(vr)) continue;
                ind = (vr - vrMin) / vStep;
                if (vr <= 0) histBuf[ind]++;
            }
        }
    }
    return float((max_element(histBuf, histBuf + (int((vrMax - vrMin) / vStep))) - histBuf)) * vStep + vrMin;
}

void calPoint(vector<POINTCLOUD> pointCloudVec,pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,float *rcsBuf,float *histBuf,size_t pointNumPerPack,float yaw)
{
    pcl::PointXYZHSV cloudPoint;
    for(size_t i = 0;i<pointCloudVec.size();i++)
    {
        for(size_t j = 0;j<pointNumPerPack;j++)
        {
            if(abs(pointCloudVec[i].point[j].range)>0&&abs(pointCloudVec[i].point[j].azi)<=80*PI/180)
            {
                pointCloudVec[i].point[j].ele = FLIPELELVATION*(pointCloudVec[i].point[j].ele+0*PI/180);

                float azi = pointCloudVec[i].point[j].azi;
                float cosEle = cos(pointCloudVec[i].point[j].ele); 
                cloudPoint.x = (pointCloudVec[i].point[j].range)*cos(azi)*cosEle; 
                cloudPoint.y = (pointCloudVec[i].point[j].range)*sin(azi)*cosEle;
                cloudPoint.z = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].ele) ; 
                // if(cloudPoint.z < -INSTALLHEIGHT)
                // {
                //    cloudPoint.z = -cloudPoint.z - 2*INSTALLHEIGHT;
                // }
                cloudPoint.h = pointCloudVec[i].point[j].doppler; 
                cloudPoint.s = pointCloudVec[i].point[j].snr;
                //rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                cloud->push_back(cloudPoint);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / vStep));
    float vrEst = hist(pointCloudVec, histBuf, yaw);
    float tmp;
    for (size_t i = 0; i < pointCloudVec.size(); i++) {
        for (size_t j = 0; j < pointNumPerPack; j++) {
            if(i*pointNumPerPack+j>=cloud->size())
            {
                break;
            }
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = (cloud->points[i * pointNumPerPack + j].h) -
                      vrEst * cos(pointCloudVec[i].point[j].azi + yaw);
                if (tmp < -errThr) {
                    cloud->points[i * pointNumPerPack + j].v = -1;
                } else if (tmp > errThr) {
                    cloud->points[i * pointNumPerPack + j].v = 1;
                } else {
                    cloud->points[i * pointNumPerPack + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    
    // ros Init
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("altosparser");

    // read files for rcs calculation
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    std::string package_path = ament_index_cpp::get_package_share_directory("altosparser");
    std::string file_path = package_path + "/data/rcs.dat";
    FILE* fp_rcs = fopen(file_path.c_str(), "rb");
    if (fp_rcs == NULL)
    {
        RCLCPP_ERROR(node->get_logger(), "[WARNING] data/rcs.dat not found in pwd [WARNING]\n");
        return -1;
    } 
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    int groupPort = 4040, uniPort = 4041;
    std::string groupIp = "224.1.2.4", localIp = "192.168.3.1";
    bool uniFlag = false;
    node->declare_parameter<std::string>("groupIp", groupIp);
    node->declare_parameter<int>("groupPort", groupPort);
    node->declare_parameter<std::string>("localIp", localIp);
    node->declare_parameter<int>("uniPort", uniPort);
    node->declare_parameter<bool>("uniFlag", uniFlag);
    node->get_parameter("groupIp", groupIp);
    node->get_parameter("groupPort", groupPort);
    node->get_parameter("localIp", localIp);
    node->get_parameter("uniPort", uniPort);
    node->get_parameter("uniFlag", uniFlag);

    int numRadar = 4;
    node->declare_parameter<int>("numRadar", numRadar);
    node->get_parameter("numRadar", numRadar);
    if (numRadar != 1 && numRadar != 4)
    {
        RCLCPP_ERROR(node->get_logger(),"numRadar is %d, must be 1 (V4) or 4 (RCU)", numRadar);
        return -1;
    }
    std::vector<RadarUnit> radars(numRadar);

    std::string baseFrameID ="base";
    node->declare_parameter<std::string>("baseFrameID", baseFrameID);
    baseFrameID = node->get_parameter("baseFrameID").as_string();

    bool sendTF;
    node->declare_parameter<bool>("sendTF", sendTF);
    node->get_parameter("sendTF", sendTF);

    for (int radarId = 0; radarId < numRadar; radarId++)
    {
        std::string paramPath = "radar" + std::to_string(radarId);
        node->declare_parameter<std::string>(paramPath + ".topicName", "radar" + std::to_string(radarId));
        node->get_parameter(paramPath + ".topicName", radars[radarId].topicName);
        if (radars[radarId].topicName.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "radar%d topic name is empty!", radarId);
            return -1;
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "radar%d topic name is %s",radarId,radars[radarId].topicName.c_str());
        }
        
        std::vector<double> tmpVec;
        node->declare_parameter<std::vector<double>>(paramPath + ".installationParam", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        tmpVec = node->get_parameter(paramPath + ".installationParam").as_double_array();
        if (tmpVec.size() != 6)
        {
            RCLCPP_ERROR(node->get_logger(), "radar%d: Invalid installation parameters! Required 6 values, got %ld.", radarId, tmpVec.size());
            return -1;
        }
        for (int i = 3; i < 6; i++)
        {
            tmpVec[i] = tmpVec[i] * PI / 180.0;
        }
        std::copy(tmpVec.begin(), tmpVec.end(), radars[radarId].installParam.begin());
    
        radars[radarId].pubCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            radars[radarId].topicName , 1);
    }

    auto markerPub = node->create_publisher<visualization_msgs::msg::Marker>("/pointNum", 10);
    auto originPub = node->create_publisher<visualization_msgs::msg::Marker>("/origin", 10);

    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    cloud->reserve(10000);

    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = baseFrameID;
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = visualization_msgs::msg::Marker::ADD;

    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    origin.scale.x = 3;
    origin.scale.y = 3;
    origin.scale.z = 3;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 0.0;
    origin.color.a = 1;

    visualization_msgs::msg::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::msg::Pose pose;
    pose.position.x = (float)-5;
    pose.position.y = 0;
    pose.position.z = 0;

    tf2_ros::TransformBroadcaster tfBr(node);
    tf2::Quaternion q;
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // socket Gen
    struct sockaddr_in  from;
    socklen_t           len = sizeof(from);
    int                 sockfd = socketGen(groupIp, groupPort, localIp, uniPort, uniFlag);
    
    // pointcloud recv para
    POINTCLOUD          pointCloudBuf;
    char*               recvBuf = (char*)&pointCloudBuf;
    size_t              pointNumPerPack = POINTNUM;
    int                 pointSizeByte = sizeof(V2Point);
    uint8_t             radarId;
    float*              histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / vStep));

    while(rclcpp::ok())
    {
        memset(recvBuf,0,sizeof(POINTCLOUD));
        int ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0, (struct sockaddr *)&from, &len);
        if (ret > 0)
		{
            radarId = pointCloudBuf.pckHeader.radar_id;
            radars[radarId].cntPointCloud = pointCloudBuf.pckHeader.length / pointSizeByte;
            uint32_t offset = pointCloudBuf.pckHeader.offset;
            if (radarId >= numRadar)
            {
                RCLCPP_ERROR(node->get_logger(), "radarId = %u in PCKHEADER.reserved >= numRadar = %u ", radarId, numRadar);
                continue; 
            }
            radars[radarId].pointCloudVec.push_back(pointCloudBuf);
            
            if ((offset / pointSizeByte + pointNumPerPack) >= radars[radarId].cntPointCloud)
            {
                calPoint(radars[radarId].pointCloudVec, cloud, rcsBuf,
                         histBuf,pointNumPerPack,radars[radarId].installParam[5]);

                // tf
                if(sendTF)
                {
                    transformStamped.header.stamp = node->get_clock()->now();
                    transformStamped.header.frame_id = baseFrameID;
                    transformStamped.child_frame_id = radars[radarId].topicName;

                    transformStamped.transform.translation.x = radars[radarId].installParam[0];
                    transformStamped.transform.translation.y = radars[radarId].installParam[1];
                    transformStamped.transform.translation.z = radars[radarId].installParam[2];

                    q.setRPY(
                        radars[radarId].installParam[3], 
                        radars[radarId].installParam[4], 
                        radars[radarId].installParam[5]
                    ); // roll, pitch, yaw
                    transformStamped.transform.rotation = tf2::toMsg(q);
                    tfBr.sendTransform(transformStamped);
                }
                
                //pub point cloud
                pcl::toROSMsg(*cloud, output);
                output.header.frame_id = radars[radarId].topicName;
                output.header.stamp = transformStamped.header.stamp;
                RCLCPP_INFO(node->get_logger(), "pointNum of %d frame of %s: %d",
                       pointCloudBuf.pckHeader.frame_id,
                       radars[radarId].topicName.c_str(),
                       radars[radarId].cntPointCloud);
                radars[radarId].pubCloud->publish(output);

                //print measure time stamp
                uint64_t sec = pointCloudBuf.pckHeader.sec;
                uint32_t nsec = pointCloudBuf.pckHeader.nsec;
                RCLCPP_INFO(node->get_logger(), "measure time stamp of %d frame of %s: %lu.%09lu",
                       pointCloudBuf.pckHeader.frame_id,
                       radars[radarId].topicName.c_str(),
                       (unsigned long)sec,
                       (unsigned long)nsec);

                origin.header.stamp = node->get_clock()->now();
                originPub->publish(origin);

                marker.header.frame_id=radars[radarId].topicName;
                marker.header.stamp = node->get_clock()->now();
                std::ostringstream str;
                str<<radars[radarId].topicName<<" pointNum: "<<radars[radarId].cntPointCloud;
                marker.text=str.str();
                marker.pose=pose;
                markerPub->publish(marker);

                // clear
                radars[radarId].pointCloudVec.clear();
                cloud->clear();
                radars[radarId].cntPointCloud = 0;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "recv failed (timeOut)   %d\n", ret);
        }
    }

    close(sockfd);
    free(histBuf);
    free(rcsBuf);
    rclcpp::shutdown();
    return -1;
}
