#ifndef MAPS_HPP
#define MAPS_HPP

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

namespace mocka {

class Maps {
public:
  // qiao
  struct Bridge {
    double bridge_width = 10.0; // 同样决定桥墩的宽度
    double bridge_length = 12.0; // 
    double bridge_height = 3.0;
    double pillar_radius = 1.0;
    double pillar_height = 8.0; // 桥墩高度，每一层大概2m会生成一圈点云
    double pillar_spacing = 5.0;
  };
  // cylinder

  typedef struct BasicInfo {
    ros::NodeHandle *nh_private;
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    sensor_msgs::PointCloud2 *output;
    pcl::PointCloud<pcl::PointXYZ> *cloud;

    // qiao
    Bridge bridge;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

public:
  Maps();

public:
  void generate(int type);

private:
  BasicInfo info;

private:
  void pcl2ros();

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();
  void addGround();
  // qiao
  void generateBridge();
};

class MazePoint {
private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace mocka

#endif // MAPS_HPP
