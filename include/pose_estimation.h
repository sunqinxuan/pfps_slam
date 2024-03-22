/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-18 17:06
#
# Filename:		pose_estimation.h
#
# Description: 
#
===============================================*/
#pragma once
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
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <limits>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <eigen3/Eigen/Eigenvalues>
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


	class PoseEstimation// : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		enum ConstraintCase { DoF_6, DoF_5, DoF_3 };

		PoseEstimation() 
		{
			max_iter_icp=10;
			max_iter_g2o=10;
			count=0;
		}

		~PoseEstimation() {}
		
		void setDebug(bool d) {debug=d;}

		// alignPlanes
		// - 
		Transform alignPlanes(std::vector<PlanePair> matched_planes);

		void align(Scan *scan, Scan *scan_ref, Transform& Tcr);

		void align(Scan *scan, Scan *scan_ref, Transform& Tcr,
					boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		// align
		// - 
		void align(Scan *scan, std::vector<PlanePair> matched_planes, Transform& Tcr,// IntrinsicParam int_param,
					boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		// align
		// - align the planes in map with the current scan observation;
		// - Tcr
		//   - input/output;
		//   - initial guess;
		//   - final transform is also stored in Tcr, apart from scan->Tcg;
		void align(Scan *scan, Map *map, Transform& Tcr,// IntrinsicParam int_param
					boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void setScanRef(Scan *s)
		{
			scan_ref=s;
		}

		void setMaxIterationICP(int i) {max_iter_icp=i;}
		void setMaxIterationLM(int i) {max_iter_g2o=i;}

	private:
		
		Scan *scan_ref;
		
		bool debug;

		int max_iter_icp;
		int max_iter_g2o;

		Eigen::Matrix3d H, H_svd_U, H_svd_V;
		Eigen::Vector3d H_singularValues;
		Eigen::Matrix<double,6,1> lambda_pln;
		Eigen::Matrix<double,6,6> q_pln;

		Transform Tcr_align_scans;
		Transform Tcr_align_planes;

		ConstraintCase constraint_case(std::vector<PlanePair> matched_planes);

		Eigen::Vector3d projectPoint2Plane(Eigen::Vector3d point, Eigen::Vector4d plane, bool vertical=false);
		Eigen::Vector3d projectPoint2Plane(Eigen::Vector3d point, Eigen::Vector4f plane);

		Eigen::Matrix3d skewSym(Eigen::Vector3d p);

		bool inViewFrustum(Map *map, Transform pose);

		int count;
	};
}

