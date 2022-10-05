#include <stdlib.h>  
#include <cmath>  
#include <limits.h>  
#include <boost/format.hpp>  
#include <fstream> 

#include <pcl/console/parse.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/visualization/point_cloud_color_handlers.h>  
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  

#include <pcl/segmentation/lccp_segmentation.h>  
#include <boost/fusion/tuple/detail/tuple.hpp>
using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

int main(int argc, char** argv)
{
	pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	pcl::PCLPointCloud2 input_pointcloud2;
	if (pcl::io::loadPCDFile("main.pcd", input_pointcloud2))
	{
		PCL_ERROR("ERROR: Could not read input point cloud ");
		return (3);
	}
	pcl::fromPCLPointCloud2(input_pointcloud2, *input_cloud_ptr);
	PCL_INFO("Done making cloud\n");

	//float voxel_resolution = 0.3f;
	//float seed_resolution = 1.2f;
	//float color_importance = 0.0f;
	//float spatial_importance = 1.0f;
	//float normal_importance = 0.0f;
	float voxel_resolution = 0.008f;
	float seed_resolution = 0.1f;
	float color_importance = 0.00f;
	float spatial_importance = 0.4f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	unsigned int k_factor = 0;

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(input_cloud_ptr);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);

	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();
	int label_max1;
	for (int i = 0; i < overseg->size(); i++) {
		if (overseg->points[i].label > label_max1)
			label_max1 = overseg->points[i].label;
		//cout << overseg->points[i].label << endl;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud1->height = 1;
	ColoredCloud1->width = overseg->size();
	ColoredCloud1->resize(overseg->size());
	for (int i = 0; i < label_max1; i++) {
		int color_R = rand() % (256) + 0;
		int color_G = rand() % (256) + 0;
		int color_B = rand() % (256) + 0;

		for (int j = 0; j < overseg->size(); j++) {
			if (overseg->points[j].label == i) {
				ColoredCloud1->points[j].x = overseg->points[j].x;
				ColoredCloud1->points[j].y = overseg->points[j].y;
				ColoredCloud1->points[j].z = overseg->points[j].z;
				ColoredCloud1->points[j].r = color_R;
				ColoredCloud1->points[j].g = color_G;
				ColoredCloud1->points[j].b = color_B;
			}
		}
	}
	pcl::io::savePCDFileASCII("OverSeg3.pcd", *ColoredCloud1);



	//float concavity_tolerance_threshold = 10;
	//float smoothness_threshold = 0.1;
	//uint32_t min_segment_size = 0;
	//bool use_extended_convexity = false;
	//bool use_sanity_criterion = false;

	float concavity_tolerance_threshold = 0.0001;
	float smoothness_threshold = 0.0001;
	uint32_t min_segment_size = 0;
	bool use_extended_convexity = true;
	bool use_sanity_criterion = true;

	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	ofstream outFile2("OverSegMerge3.txt", std::ios_base::out);
	for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
		outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << endl;
	}

	int label_max2 = 0;
	for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label > label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
			//lccp_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud2->height = 1;
	ColoredCloud2->width = lccp_labeled_cloud->size();
	ColoredCloud2->resize(lccp_labeled_cloud->size());
	for (int i = 0; i < label_max2; i++) {
		int color_R = rand() % (256) + 0;
		int color_G = rand() % (256) + 0;
		int color_B = rand() % (256) + 0;

		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {
			if (lccp_labeled_cloud->points[j].label == i) {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
			}
		}
	}
	pcl::io::savePCDFileASCII("cluster3.pcd", *ColoredCloud2);
	return 0;
}