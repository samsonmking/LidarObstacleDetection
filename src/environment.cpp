/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <pcl/visualization/pcl_visualizer.h>

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0);
  auto pointCloud = lidar->scan();
  // renderRays(viewer, lidar->position, pointCloud);

  // TODO:: Create point processor
  auto processPointClould = new ProcessPointClouds<pcl::PointXYZ>();
  auto segmentCloud = processPointClould->SegmentPlane(pointCloud, 100, 0.2);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClould->Clustering(segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0.5, 0.5, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    processPointClould->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
    Box box = processPointClould->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }

  renderPointCloud(viewer, segmentCloud.second, "groundPlane", Color(0, 1, 0));
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  auto filterCloud = pointProcessor->FilterCloud(inputCloud, 0.15, Eigen::Vector4f(-25.0, -5.0, -2.0, 1.0), Eigen::Vector4f(25.0, 7.0, 0.5, 1.0));
  auto segmentCloud = pointProcessor->SegmentPlane(filterCloud, 100, 0.2);
  auto cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.6, 30, 1000);

  std::vector<Color> colors = {Color(1, 0, 0), Color(0.5, 0.5, 0), Color(0, 0, 1)};

  for (int i = 0; i < cloudClusters.size(); ++i) {
    std::cout << "cluster size ";
    auto cluster = cloudClusters[i];
    pointProcessor->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(i), colors[i % 3]);
    Box box = pointProcessor->BoundingBox(cluster);
    renderBox(viewer, box, i);
  }

  renderPointCloud(viewer, segmentCloud.second, "groundPlane", Color(0, 1, 0));
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if (streamIterator == stream.end()) streamIterator = stream.begin();

    viewer->spinOnce();
  }
}