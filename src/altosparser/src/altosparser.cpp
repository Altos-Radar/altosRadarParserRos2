#include "altos-pc-v2.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <array>
using namespace std;
#define vrMax 60
#define vrMin -60
#define vStep 0.1
#define errThr 3
#define PI 3.1415926
#define GROUPIP "224.1.2.4"
#define GROUPPORT 4040
#define LOCALIP "192.168.3.1"
#define UNIPORT 4041
#define UNIFLAG 0
#define FLIPELELVATION -1
#define INSTALLHEIGHT 1.85

struct RadarUnit
{
    std::string topicName;
    std::array<double, 6> installParam;
    ros::Publisher pubCloud;

    vector<POINTCLOUD> pointCloudVec;

    int cntPointCloud = 0;
};

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

int socketGen()
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
    if(UNIFLAG)
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UNIPORT);
        addr.sin_addr.s_addr = inet_addr(LOCALIP);
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return -1;
        }
    }else
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(GROUPPORT);
        addr.sin_addr.s_addr = INADDR_ANY;
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return -1;
        }

        req.imr_multiaddr.s_addr = inet_addr(GROUPIP);
        req.imr_interface.s_addr = inet_addr(LOCALIP);
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

    for (int i = 0; i < pointCloudVec.size(); i++)
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

void calPoint(vector<POINTCLOUD> pointCloudVec,pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,float *rcsBuf,float *histBuf,int pointNumPerPack,float yaw)
{
    pcl::PointXYZHSV cloudPoint;
    for(int i = 0;i<pointCloudVec.size();i++)
    {
        for(int j = 0;j<pointNumPerPack;j++)
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
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < pointNumPerPack; j++) {
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
    
    // read files for rcs calculation
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    if (fp_rcs == NULL)
    {
        ROS_ERROR("[WARNING] data/rcs.dat not found in pwd [WARNING]\n");
        return -1;
    } 
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    // ros Init
    ros::init(argc, argv, "altosParser");
    ros::NodeHandle nh;

    int numRadar = 4;
    nh.getParam("altosParserParameters/numRadar", numRadar);
    if (numRadar != 1 && numRadar != 4)
    {
        ROS_ERROR("numRadar is %d, must be 1 (V4) or 4 (RCU)", numRadar);
        return -1;
    }
    std::vector<RadarUnit> radars(numRadar);

    std:string baseFrameID ="base";
    nh.getParam("altosParserParameters/baseFrameID", baseFrameID);

    bool sendTF;
    nh.getParam("altosParserParameters/sendTF", sendTF);

    for (int radarId = 0; radarId < numRadar; radarId++)
    {
        std::string paramPath = "altosParserParameters/radar" + std::to_string(radarId);
        nh.getParam(paramPath + "/topicName", radars[radarId].topicName);
        if (radars[radarId].topicName.empty())
        {
            ROS_ERROR("radar%d topic name is empty!", radarId);
            return -1;
        }

        std::vector<double> tmpVec;
        nh.getParam(paramPath + "/installationParam", tmpVec);
        if (tmpVec.size() != 6)
        {
            ROS_ERROR("radar%d: Invalid installation parameters! Required 6 values, got %ld.", radarId, tmpVec.size());
            return -1;
        }
        for (int i = 3; i < 6; i++)
        {
            tmpVec[i] = tmpVec[i] * PI / 180.0;
        }
        std::copy(tmpVec.begin(), tmpVec.end(), radars[radarId].installParam.begin());
    
        radars[radarId].pubCloud = nh.advertise<sensor_msgs::PointCloud2>(
            radars[radarId].topicName , 1);
    }

    ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
    ros::Publisher originPub = nh.advertise<visualization_msgs::Marker>("origin", 10);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    cloud->reserve(10000);

    visualization_msgs::Marker origin;
    origin.header.frame_id = baseFrameID;
    origin.type = visualization_msgs::Marker::SPHERE;
    origin.action = visualization_msgs::Marker::ADD;

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

    visualization_msgs::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::Pose pose;
    pose.position.x =  (float)-5;
    pose.position.y =  0;
    pose.position.z =0;

    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster tfBr;
    
    // socket Gen
    struct sockaddr_in  from;
    socklen_t           len = sizeof(from);
    int                 sockfd = socketGen();
    
    // pointcloud recv para
    POINTCLOUD          pointCloudBuf;
    char*               recvBuf = (char*)&pointCloudBuf;
    int                 pointNumPerPack = POINTNUM;
    int                 pointSizeByte = sizeof(V2Point);
    int                 recvFrameLen = 0;
    float               vrEst = 0;
    uint8_t             radarId;
    unsigned char       mode;
    float*              histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / vStep));

    while(ros::ok())
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
                ROS_ERROR("radarId = %u in PCKHEADER.reserved >= numRadar = %u ", radarId, numRadar);
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
                    transform.setOrigin(
                        tf::Vector3(
                            radars[radarId].installParam[0],
                            radars[radarId].installParam[1], 
                            radars[radarId].installParam[2]
                        )
                    ); // x, y, z
                    q.setRPY(
                        radars[radarId].installParam[3], 
                        radars[radarId].installParam[4], 
                        radars[radarId].installParam[5]
                    ); // roll, pitch, yaw
                    transform.setRotation(q);
                    tfBr.sendTransform(
                        tf::StampedTransform(
                            transform,
                            ros::Time::now(),
                            baseFrameID,
                            radars[radarId].topicName
                        )
                    );
                }
                
                //pub point cloud
                pcl::toROSMsg(*cloud, output);
                output.header.frame_id = radars[radarId].topicName;
                output.header.stamp = ros::Time::now();
                ROS_INFO("pointNum of %d frame of %s: %d\n",
                       pointCloudBuf.pckHeader.frame_id,
                       radars[radarId].topicName.c_str(),
                       radars[radarId].cntPointCloud);
                output.header.stamp = ros::Time::now();
                radars[radarId].pubCloud.publish(output);

                //print measure time stamp
                uint64_t sec = pointCloudBuf.pckHeader.sec;
                uint32_t nsec = pointCloudBuf.pckHeader.nsec;
                double measureTime = (double)sec + (double)nsec / 1e9;
                ROS_INFO("measure time stamp of %d frame of %s: %lu.%09lu\n",
                       pointCloudBuf.pckHeader.frame_id,
                       radars[radarId].topicName.c_str(),
                       (unsigned long)sec,
                       (unsigned long)nsec);

                originPub.publish(origin);

                marker.header.frame_id=radars[radarId].topicName;
                marker.header.stamp = ros::Time::now();
                ostringstream str;
                str<<radars[radarId].topicName<<" pointNum: "<<radars[radarId].cntPointCloud;
                marker.text=str.str();
                marker.pose=pose;
                markerPub.publish(marker);

                // clear
                radars[radarId].pointCloudVec.clear();
                cloud->clear();
                radars[radarId].cntPointCloud = 0;
            }
        } else {
            ROS_ERROR("recv failed (timeOut)   %d\n", ret);
        }
    }

    close(sockfd);
    free(histBuf);
    free(rcsBuf);
    return -1;
}
