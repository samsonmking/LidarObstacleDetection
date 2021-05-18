// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <memory>

// Using templates for KDTree so help out the linker
#include "kdtree.cpp"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                                                              Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloud_filtered);

  typename pcl::CropBox<PointT> crop(true);
  crop.setMin(minPoint);
  crop.setMax(maxPoint);
  crop.setInputCloud(cloud_filtered);
  crop.filter(*cloud_cropped);

  std::vector<int> roofIndicies;
  typename pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_cropped);
  roof.filter(roofIndicies);

  pcl::PointIndices::Ptr roofInliers(new pcl::PointIndices);
  for (int point : roofIndicies) {
    roofInliers->indices.push_back(point);
  }

  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_cropped);
  extract.setIndices(roofInliers);
  extract.setNegative(true);
  extract.filter(*cloud_cropped);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud_cropped;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);
  extract.filter(*planeCloud);

  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices& inliersIndicies, int maxIterations,
                                        float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  for (int i = 0; i < maxIterations; i++) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      int randIndex = rand() % (cloud->points.size());
      if (inliers.count(randIndex) == 0) {
        inliers.insert(randIndex);
      }
    }
    auto itr = inliers.begin();
    auto p1 = cloud->points[*itr];
    itr++;
    auto p2 = cloud->points[*itr];
    itr++;
    auto p3 = cloud->points[*itr];
    pcl::PointXYZ v1 = {(p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z)};
    pcl::PointXYZ v2 = {(p3.x - p1.x), (p3.y - p1.y), (p3.z - p1.z)};

    // cross product: v1 x v2 = <a, b, c>
    float a = (v1.y * v2.z) - (v1.z * v2.y);
    float b = (v1.z * v2.x) - (v1.x * v2.z);
    float c = (v1.x * v2.y) - (v1.y * v2.x);
    float d = -1.0 * ((a * p1.x) + (b * p1.y) + (c * p1.z));

    for (int p = 0; p < cloud->points.size(); p++) {
      if (inliers.count(p) > 0) {
        continue;
      }
      auto point = cloud->points[p];
      float dist = fabs((a * point.x) + (b * point.y) + (c * point.z) + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
      if (dist <= distanceTol) {
        inliers.insert(p);
      }
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  for (int index : inliersResult) {
    inliersIndicies.indices.push_back(index);
  }
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  Ransac(cloud, *inliers, maxIterations, distanceThreshold);

  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float clusterTolerance, int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  auto tree = std::make_shared<KDTree<PointT>>();

  for (int id = 0; id < cloud->points.size(); ++id) {
    tree->insert(cloud->points[id], id);
  }

  std::vector<std::vector<int>> cluster_indicies = euclideanCluster(cloud, tree, clusterTolerance);

  for (std::vector<int> cin : cluster_indicies) {
    if (cin.size() < minSize || cin.size() > maxSize) {
      continue;
    }
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (int in : cin) {
      cloud_cluster->push_back((*cloud)[in]);
    }
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                           std::shared_ptr<KDTree<PointT>> tree, float distanceTol) {
  std::vector<std::vector<int>> clusters;
  std::unordered_set<int> seen;

  for (int i = 0; i < cloud->points.size(); i++) {
    if (seen.count(i) == 0) {
      clusters.push_back(proximity(cloud, i, tree, distanceTol, seen));
    }
  }
  return clusters;
}

template <typename PointT>
std::vector<int> ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int index, std::shared_ptr<KDTree<PointT>> tree,
                                                       float distanceTol, std::unordered_set<int>& seen) {
  std::vector<int> cluster;
  seen.insert(index);
  cluster.push_back(index);

  auto nearbyPoints = tree->search(cloud->points[index], distanceTol);
  for (auto nearbyIndex : nearbyPoints) {
    if (seen.count(nearbyIndex) == 0) {
      cluster.push_back(nearbyIndex);
      auto nextCluster = proximity(cloud, nearbyIndex, tree, distanceTol, seen);
      cluster.insert(cluster.end(), nextCluster.begin(), nextCluster.end());
    }
  }
  return cluster;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}