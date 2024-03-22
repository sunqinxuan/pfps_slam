/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-07-18 16:50
#
# Filename: plane_feature_extraction.h
#
# Description: 
#
===============================================*/


#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl-1.8/pcl/common/centroid.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <limits>
#include "ANN/ANN.h"
#include "types.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ulysses
{
	class PlaneFeatureExtraction : public pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>, 
									   public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		PlaneFeatureExtraction() {}
		~PlaneFeatureExtraction() {}

//		void loadScan(Scan *s) {scan=s;}

		// initialize
		// - some default arguments set for the plane segmentation and the edge detection;
		void initialize();

		void setDebug(bool d) {debug=d;}


		// generateObservation
		// - generate all the needed information for plane feature;
		// - scan->Tcg is allocated and set to identity here;
		void generateObservation(Scan *scan);

		// generatePlaneFeature
		// - scan->observed_planes;
		// - after pose estimation (scan->Tcg required);
		// - all the plane coefficients and point coordinates are in local frame;
		void generatePlaneFeature(Scan *scan);

	private:
		
		bool debug;
		// allocated before load in;
		//Scan *scan;
	
		// segmentPlanes
		// - plane segmentation;
		// - save to scan
		//   + planes
		//     - planar_regions
		//     - plane_indices
		void segmentPlanes(Scan *scan);

		// extractEdges
		// - extract edge points;
		// - save to scan
		//   + edge
		//     - edge_label_cloud
		//     - edge_indices
		void extractEdges(Scan *scan);

		// associateEdges
		// - associate occluding and occluded points;
		// - projective_rays = occluded - occluding;
		void associateEdges(Scan *scan);

		// associateEdgePlane
		// - associate edge points with plane segments;
		// - edge_corresponding_planes;
		void associateEdgePlane(Scan *scan);	
	};
}

