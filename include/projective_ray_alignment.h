/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-05-09 09:39
#
# Filename: projective_ray_alignment.h
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
#include <pcl-1.8/pcl/registration/transforms.h>
#include <limits>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "ANN/ANN.h"
#include "types.h"

namespace ulysses
{
	// EdgeSE3pra
	// - define the edge for the graph optimization;
	// - unary edge;
	// - vertex: Tcw (inverse of camera pose);
	// - measurement: edge point (occluding) coordinates;
	class EdgeSE3pra: public g2o::BaseUnaryEdge< 3, Eigen::Vector3d, g2o::VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeSE3pra() {debug=false;}

		EdgeSE3pra ( Eigen::Vector3d point)//, Eigen::Vector3d point_cur) // Scan *s, ANNkd_tree *kd, cv::Mat* image )
			: p_global ( point )//, p_cur (point_cur) // scan ( s ), kdtree(kd), depth_image ( image )
		{
			int_param=IntrinsicParam();
			debug=false;
		}
		
		EdgeSE3pra ( Eigen::Vector3d point, float fx, float fy, float cx, float cy) //, Scan *s, ANNkd_tree *kd, cv::Mat* image)
			: p_global ( point )//, p_cur (point_cur) //, scan ( s ), kdtree(kd), depth_image ( image )
		{
			int_param=IntrinsicParam(fx,fy,cx,cy);
			debug=false;
		}

		// compute the error function;
		// i.e., the e in F=eT*\Omega*e;
		virtual void computeError();

		// compute the jacobian _jacobianOplusXi for the state increment;
//		virtual void linearizeOplus();

		// dummy read and write functions because we don't care...
		virtual bool read ( std::istream& in ) {}
		virtual bool write ( std::ostream& out ) const {}
		
		void setDebug(bool d) {debug=d;}


	private:

		bool debug;

		// point in global frame;
		Eigen::Vector3d p_global;
		// point in current frame;
//		Eigen::Vector3d p_cur;

		// camera intrinsic parameters;
		IntrinsicParam int_param;

		// reference depth image;
//		cv::Mat* depth_image=nullptr;

		// scan
		// - reference scan;
		// - allocated before load in;
//		Scan *scan;

		// kdtree
		// - build from the occluding edge points in current scan;
//		ANNkd_tree *kdtree;

		// getPixelDepth
		// - get the depth value at (u,v) of depth_image;
//		double getPixelDepth(int u,int v);
	};


	class ProjectiveRayAlignment : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		ProjectiveRayAlignment() {}
		~ProjectiveRayAlignment() {}

//		void loadScan(Scan *s) {scan=s;}
//
// 		void getEdgePoints(pcl::PointCloud<pcl::Label>::Ptr l, std::vector<pcl::PointIndices>& li)
//		{
//			l=scan->edge_label_cloud;
//			li=scan->edge_indices;
//		}

		// initialize
		// - some default arguments set for the edge detection;
		void initialize()
		{
			float th_dd = 0.04f;
			int max_search = 100;
			// edge detection;
			setDepthDisconThreshold (th_dd);
			setMaxSearchNeighbors (max_search);
			setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED);
		}

		// extract
		// - edge point detection;
		//   - of the whole image;
		//   - no correspondence between occluding and occluded;
		//   - no correspondence between salient points and the plane;
		// - save to scan
		//   + edge
		//     - edge_label_cloud
		//     - edge_indices
		void extractEdgePoints(Scan *scan);

		// align
		// - 
		void align(Scan *scan_ref, Scan *scan_cur, Eigen::Isometry3d& Tcr, IntrinsicParam int_param,
					boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:
		
		// scan
		// - reference scan;
		// - allocated before load in;
//		Scan *scan;

		// for edge detection;
//		pcl::PointCloud<pcl::Label> labels;
//		std::vector<pcl::PointIndices> edge_indices;
	};
}

