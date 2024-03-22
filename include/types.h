/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-12 10:55
#
# Filename:		types.h
#
# Description: some basic structures.
#
===============================================*/
#pragma once
#include <stdio.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <opencv2/core/core.hpp>

namespace ulysses
{
	// IntrinsicParam 
	// - intrinsic parameters of camera;
	// - fx, fy, cx, cy;
	// - pixel size not defined.
	struct IntrinsicParam
	{
		IntrinsicParam() // {fx=525;fy=525;cx=240;cy=320;}
		{
			fx=517.3;//591.1;//567.6;//580.8;//525;
			fy=516.5;//590.1;//570.2;//581.8;//525;
			cx=318.6;//331;//324.7;//308.8;//319.5;
			cy=255.3;//234;//250.1;//253;//239.5;
		}
		IntrinsicParam(double fx_, double fy_, double cx_, double cy_) {fx=fx_;fy=fy_;cx=cx_;cy=cy_;}
//		IntrinsicParam(const IntrinsicParam int_param) {fx=int_param.fx;fy=int_param.fy;cx=imt_param.cx;cy=int_param.cy;}

		double fx,fy,cx,cy;

		// getMatrix
		// - K =
		// - |fx 0  cx|
		//   |0  fy cy|
		//   |0  0  1 |
		Eigen::Matrix3d getMatrix()
		{
			Eigen::Matrix3d mat;
			mat.setIdentity();
			mat(0,0)=fx;
			mat(1,1)=fy;
			mat(0,2)=cx;
			mat(1,2)=cy;
			return mat;
		}

		// project
		// - project the point in 3d camera frame into the pixel frame;
		// - u=Kp;
		// - u is the homogeneous coordinates;
		// - u=[col,row,1]^T;
		Eigen::Vector3d project(Eigen::Vector3d p)
		{
			Eigen::Vector3d tmp,u;
			tmp=getMatrix()*p/p(2);
			u(0)=tmp(0);
			u(1)=tmp(1);
			u(2)=1;
			//u(0) = p[0]*fx/p[2] + cx;
			//u(1) = p[1]*fy/p[2] + cy;
			return u;
		}
	};

	struct Plane;

	struct SalientPoint
	{
		enum TYPE {OCCLUDING,OCCLUDED,HIGH_CURVATURE};
		TYPE type;

		// xyz - 3d coordinate in world frame;
		// rgb - color;
		Eigen::Vector3d xyz,rgb;

		// proj_ray
		// - vector from this point
		// - to its corresponding occluding/occluded point;
		Eigen::Vector3d proj_ray;

		// ptr_plane
		// - point to corresonding occluding/occluded point
		// - on another plane;
		SalientPoint *ptr_plane;
		// ptr_nn
		// - point to nearest neighbor in the map;
		// - OCCLUDING - xyz;
		// - OCCLUDED  - xyz + proj_ray;
		SalientPoint *ptr_nn;

		// plane_on
		// - the plane this point is on;
		Plane *plane_on;
	};
//	struct SalientPoint;

	struct Transform
	{
		Transform()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform(const Transform& T)
		{
			R=T.R;
			t=T.t;
		}

		Transform(Eigen::Matrix3d R_, Eigen::Vector3d t_)
		{
			R=R_;
			t=t_;
		}

		Transform(Eigen::Quaterniond Q_, Eigen::Vector3d t_)
		{
			R=Q_.toRotationMatrix();
			t=t_;
		}

		Eigen::Vector3d t;
		Eigen::Matrix3d R;

		IntrinsicParam intrinsic;

		std::vector<Plane*> ptr_planes;

		// eular angle: Z-Y-X
		Eigen::Vector3d Eulars() {return R.eulerAngles(2,1,0);}

		Eigen::Quaterniond Quat() {return Eigen::Quaterniond(R);}

		void inverse()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			R=R_tmp;
			t=t_tmp;
		}

		Transform inv()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			Transform T(R_tmp,t_tmp);
			return T;
		}

		void setIdentity()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		void leftMultiply(Transform T)
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=T.R*R;
			t_tmp=T.R*t+T.t;
			R=R_tmp;
			t=t_tmp;
		}

		Transform leftMul(Transform T)
		{
			Transform TT;
			TT.R=T.R*R;
			TT.t=T.R*t+T.t;
			return TT;
		}

		void leftMultiply(Eigen::Isometry3d T)
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=T.rotation()*R;
			t_tmp=T.rotation()*t+T.translation();
			R=R_tmp;
			t=t_tmp;
		}

		Eigen::Vector3d transform(Eigen::Vector3d p)
		{
			return R*p+t;
		}

		void transform(SalientPoint *p)
		{
			p->xyz=transform(p->xyz);
			p->proj_ray=R*p->proj_ray;
		}

//		// inv_transform
//		// - return T.inv()*p;
//		Eigen::Vector3d inv_transform(Eigen::Vector3d p)
//		{
//			return R.transpose()*p-R.transpose()*t;
//		}

		Eigen::Vector4d transformPlane(Eigen::Vector4d pln)
		{
			Eigen::Vector4d p;
			p.block<3,1>(0,0)=R*pln.block<3,1>(0,0);
			p(3)=pln(3)-p.block<3,1>(0,0).transpose()*t;
			return p;
		}

//		Eigen::Vector4d inv_transformPlane(Eigen::Vector4d pln)
//		{
//			Eigen::Vector4d p;
//			p.block<3,1>(0,0)=R.transpose()*pln.block<3,1>(0,0);
//			p(3)=pln(3)-p.block<3,1>(0,0).transpose()*R.transpose()*t;
//			return p;
//		}

		Eigen::Matrix4f getMatrix4f()
		{
			Eigen::Matrix4f transform;
			transform.setIdentity();
			for(int i=0;i<3;i++)
			{
				transform(i,3)=(float)t(i);
				for(int j=0;j<3;j++)
				{
					transform(i,j)=(float)R(i,j);
				}
			}
			return transform;
		}
	};


	struct Plane
	{
		Eigen::Vector4d coefficients;
		Eigen::Vector3d centroid_xyz, centroid_rgb;
		Eigen::Matrix3d covariance_xyz, covariance_rgb;

		Eigen::Vector3d eigenvalues_xyz;
		Eigen::Matrix3d eigenvectors_xyz;
		Eigen::Vector3d normal;

		unsigned int count;
		double size;
//		pcl::PointCloud<pcl::PointXYZRGBA> contour;
		std::vector<Eigen::Vector3d> contour;
		double curvature;
		int index;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points;

		// salient_points
		std::vector<Eigen::Vector3d> real_contour;
		std::vector<std::vector<SalientPoint*> > occluding_points_seq;
		std::vector<std::vector<SalientPoint*> > occluded_points_seq;

		// occluding_points_plane_seq
		// - indices of occluding_points_seq;
		// - correspond to occluding points projecting to another plane;
//		std::vector<std::vector<int> > occluding_points_plane_seq;

		std::vector<Transform*> ptr_camera_pose_seq;

		double distancePoint(Eigen::Vector3d point)
		{
			double dist;
			dist=fabs(coefficients.block<3,1>(0,0).dot(point)+coefficients(3))/coefficients.block<3,1>(0,0).norm();
			return dist;
		}

		// distancePoint2Plane
		// - distance from occluded points in this plane to p;
		double distancePoint2Plane(Plane *p)
		{
			if(occluding_points_seq[0].size()+occluded_points_seq[0].size()==0)
				return 0;
			double dist=0;
			for(size_t i=0;i<occluding_points_seq[0].size();i++)
			{
				dist+=p->distancePoint(occluding_points_seq[0][i]->xyz);
			}
			for(size_t i=0;i<occluded_points_seq[0].size();i++)
			{
				dist+=p->distancePoint(occluded_points_seq[0][i]->xyz);
			}
			dist/=occluded_points_seq[0].size()+occluding_points_seq[0].size();
			return dist;
		}

		double similarity_geom(Plane *p)
		{
			// color similarity;
			// Bhattachryya distance bwteen the color distribution of two planes;
			Eigen::Matrix3d C=(covariance_xyz+p->covariance_xyz)/2;
//			std::cout<<C<<std::endl;
//			std::cout<<"covariance"<<std::endl<<covariance_xyz<<std::endl;
//			std::cout<<"covariance"<<std::endl<<p->covariance_xyz<<std::endl;
			
			Eigen::Vector3d m=centroid_xyz-p->centroid_xyz;
//			std::cout<<m.transpose()<<std::endl;
			Eigen::Matrix<double,1,1> M_dist=m.transpose()*C.inverse()*m/8;
//			std::cout<<M_dist<<std::endl;
//			std::cout<<C.determinant()<<", "<<covariance_xyz.determinant()<<", "<<p->covariance_xyz.determinant()<<std::endl;
			double s_geo=M_dist(0,0)+0.5*log(fabs(C.determinant())/sqrt(fabs(covariance_xyz.determinant())*fabs(p->covariance_xyz.determinant())));
//			std::cout<<s_geo<<std::endl;
			
			return s_geo;
		}

		double similarity_geom(Eigen::Vector3d centroid, Eigen::Matrix3d covariance)
		{
			EIGEN_ALIGN16 Eigen::Vector3d eigen_value;
			EIGEN_ALIGN16 Eigen::Matrix3d eigen_vector;
			pcl::eigen33 (covariance, eigen_vector, eigen_value);
			Eigen::Vector3d delta_centroid=centroid-centroid_xyz;
			double dist_centroid=delta_centroid.norm();
//			std::cout<<"dist_centroid - "<<dist_centroid<<std::endl;
			Eigen::Vector3d n1=eigenvectors_xyz.block<3,1>(0,0);
			if(n1.dot(centroid_xyz)>0)
				n1=-n1;
			Eigen::Vector3d n2=eigen_vector.block<3,1>(0,0);
			if(n2.dot(centroid)>0)
				n2=-n2;
			double dist_normal=n1.transpose()*n2;
//			std::cout<<"dist_normal - "<<dist_normal<<std::endl;
			double dist_area=fabs(M_PI*sqrt(eigen_value(1))*sqrt(eigen_value(2))-M_PI*sqrt(eigenvalues_xyz(1))*sqrt(eigenvalues_xyz(2)));
//			std::cout<<"dist_area - "<<dist_area<<std::endl;

			double s_geo=dist_centroid;//+dist_normal+dist_area;
			// Bhattachryya distance bwteen the color distribution of two planes;
//			Eigen::Matrix3d C=(covariance_xyz+covariance)/2;
//			Eigen::Vector3d m=centroid_xyz-centroid;
//			Eigen::Matrix<double,1,1> M_dist=m.transpose()*C.inverse()*m/8;
//			double s_geo=M_dist(0,0);//+0.5*log(fabs(C.determinant())/sqrt(fabs(covariance_xyz.determinant())*fabs(covariance.determinant())));
			
			return s_geo;
		}

		double similarity_color(Plane *p)
		{
			// color similarity;
			// Bhattachryya distance bwteen the color distribution of two planes;
			Eigen::Matrix3d C=(covariance_rgb+p->covariance_rgb)/2;
//			std::cout<<C<<std::endl;
//			std::cout<<C.determinant()<<", "<<covariance_rgb.determinant()<<", "<<p->covariance_rgb.determinant()<<std::endl;
			Eigen::Vector3d m=centroid_rgb-p->centroid_rgb;
			Eigen::Matrix<double,1,1> M_dist=m.transpose()*C.inverse()*m/8;
			double s_col=M_dist(0,0)+0.5*log(C.determinant()/sqrt(covariance_rgb.determinant()*p->covariance_rgb.determinant()));
//			std::cout<<s_col<<std::endl;
			
			return s_col;
		}

		double similarity_angle(Plane *p)
		{
			// angle between two normals;
			double dot=coefficients.block<3,1>(0,0).transpose()*p->coefficients.block<3,1>(0,0);
			if(dot>1)
				dot=1;
			double s_ang=acos(dot);
			return s_ang;
		}
		
		double similarity_dist(Plane *p)
		{
			double dist=fabs(coefficients(3)-p->coefficients(3));
			return dist;
		}

		bool similar(Plane *p, double thresh_angle, double thresh_color, double thresh_dist)
		{
			for(size_t i=0;i<p->occluding_points_seq[0].size();i++)
			{
//				distPoint(p->occluding_points_seq[0][i]->xyz)
			}
			if(similarity_angle(p)<thresh_angle*0.0174 && similarity_color(p)<thresh_color && similarity_dist(p)<thresh_dist)
				return true;
			else
				return false;
		}

		bool similar(Plane *p)
		{
			return similar(p, 10.0, 0.2, 0.1);
		}
	};

	struct Scan
	{
		Transform *Tcg;//, Tcr;
		Transform *Tcg_gt;

//		std::vector<Plane*> planes;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
		pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;
		cv::Mat img_rgb, img_depth;
		
		std::vector<Plane*> observed_planes;


		// edge_label_cloud
		// - label the edge points;
		pcl::PointCloud<pcl::Label>::Ptr edge_label_cloud;

		// edge_indices
		// - indices of edge points;
		// - corresponding to edge labels;
		//   - edge_indices[0], nan_boundary_edges;
		//   - edge_indices[1], occluding_edges;
		//   - edge_indices[2], occluded_edges;
		//   - edge_indices[3], high_curvature_edges;
		//   - edge_indices[4], rgb_edges;
		std::vector<pcl::PointIndices> edge_indices;

		// salient_points
		// - 
//		std::vector<std::vector<SalientPoint*> > salient_points;

		// edge_corresponding_plane
		// - which plane segment each edge point was on;
		// - corresponding to edge_indices;
		// - planar_regions[edge_corresponding_plane[i].indices[j]];
		// - edge_corresponding_plane[i].indices[j]=-1 - in the nonplanar region;
		std::vector<pcl::PointIndices> edge_corresponding_plane;



		// occluding_correspondences
		// - corresponding occluded point to the occluding point;
		// - edge_indices[2].indices[scan->occluding_correspondences.indices[j]];
		pcl::PointIndices occluding_correspondences;
		// occluding_correspondences_distance
		// - the distance on the image domain;
		// - between pixels;
		std::vector<double> occluding_correspondences_distance;
		// projective_ray
		// - ray from occluding point to occluded point;
		std::vector<Eigen::Vector3d> projective_rays;


		// occluded_correspondences
		// - corresponding occluding point to the occluded ones;
		pcl::PointIndices occluded_correspondences;



		// planar_regions
		// - centroid, covariance, count, contour, coefficients(ax+by+cy+d=0);
		std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > planar_regions;

		// plane_indices
		// - indices of points on each plane;
		// - corresponding to planar_regions;
		std::vector<pcl::PointIndices> plane_indices;

		// plane_label_cloud
		// - 0: non-plnanar region;
		// - i: planar_regions[i-1];
		pcl::PointCloud<pcl::Label>::Ptr plane_label_cloud;



		// getDepth
		// - get the depth value at pixel (u,v) of img_depth;
		double getDepth(int u, int v)
		{
			unsigned char *depth_data_ptr=img_depth.data+img_depth.step[0]*u+img_depth.step[1]*v;
			unsigned short *depth_ptr=new unsigned short;
			memcpy(depth_ptr,depth_data_ptr,2);
			return (double)*depth_ptr/5000;
		}

		// getDepth
		// - get the depth value at index;
		double getDepth(int index)
		{
			int u=index/img_depth.cols;
			int v=index%img_depth.cols;
			return getDepth(u,v);
		}
	};

	struct Map
	{
		std::vector<Plane*> planes;
		std::vector<Transform*> camera_poses;
	};


//	struct Pixel
//	{
//		// pixel coordinate;
//		int u,v;
//		// number of pixels in a row;
//		int step;
//
//		Pixel() {u=0;v=0;step=640;}
//		Pixel(int i,int j) {u=i;v=j;step=640;}
//		Pixel(int i,int j,int s) {u=i;v=j;step=s;}
//
//		int getIndex() {return u*step+v;}
//
//		// to use it as the index in std::map structure;
//		bool operator < (const Pixel &x) const
//		{
//			if(this->u*this->step+this->v<x.u*x.step+x.v)
//				return true;
//			else
//				return false;
//		}
//	};

	// index in the pps (theta-phy-d)
	//struct Pixel_pps
	//{
	//	int theta,phy,d,layer;
	//	int step_theta,step_phy;
	//	bool operator < (const Pixel_pps &x) const
	//	{
	//		if(this->layer==x.layer)
	//		{
	//			if(this->d*this->step_theta*this->step_phy+this->theta*this->step_phy+this->phy < x.d*x.step_theta*x.step_phy+x.theta*x.step_phy+x.phy)
	//				return true;
	//			else
	//				return false;
	//		}
	//		else if(this->layer<x.layer)
	//			return true;
	//		else
	//			return false;
	//	}
	//};

//	struct Point
//	{
//		// pps: coordinate in the PPS for the local plane parameters (after Rotation2eigen);
//		// rgb: [0,1] RGB information of the point;
//		// xyz: coordinate in the camera coordinate system;
//		// normal: local plane normal in original camera coordinate system;
//		Eigen::Vector3d pps, rgb, xyz, normal;
//
//		// u,v: pixel coordinate;
//		int u,v; // u<480, v<640
//
//		// cov: covariance for icp;
//		Eigen::Matrix3d cov; 
//		double weight;
//	};

//	struct Cell
//	{
//		// number of points in the cell;
//		int num_points;
//
//		// normal,d: plane coordinate in the camera coordinate system;
//		Eigen::Vector3d normal;
//		double d;
//
//		// avg_pps,cov_pps: computed from the PPS coordinates of the inside points;
//		// avg_rgb,cov_rgb: computed from the RGB of the inside points;
//		Eigen::Vector3d avg_pps, avg_rgb;
//		Eigen::Matrix3d cov_pps, cov_rgb;
//		
//		bool isEmpty; // whether the cell is empty;
//		bool isBottom; // whether the cell is on bottom level;
//
//		// points in the cell;
//		std::vector<Point> points_in;
//
//		// indices w.r.t. point_cloud and normal_cloud;
//		// to index the cell points in the point_cloud;
//		pcl::PointIndices::Ptr inliers;
//
//		// the followings are used in the sting
//		// temprary existence
//		// deleted when PlaneExtraction_STING.cpp is modified
//		//int layer;
//		//bool relevant;
//		//Cell *father;
//		//Cell *child[8];
//	};

//	struct Plane
//	{
//		// plane parameters;
//		Eigen::Vector3d normal;
//		double d;
//		Eigen::Vector4d pro_para;
//		
//		// center point;
//		Point pc;
//
//		// size of the plane;
//		//double size;
//		
//		// avg_pps, avg_rgb, cov_pps, cov_rgb: computed from the points_in
//		// avg_pps, cov_pps: after Rotation2eigen
//		Eigen::Vector3d avg_pps, avg_rgb;
//		Eigen::Matrix3d cov_pps, cov_rgb;
//
//		// not used temporarily;
//		Eigen::Vector3d pps, rgb;
//		
//		// all the points on plane;
//		std::vector<Point> points_in;
//		// number of points on plane;
//		int num_points;
//		
//		// pointer to the salient points;
//		std::vector<Point*> ps;
//		std::vector<Eigen::Vector3d> ps_v;
//		std::vector<double> ps_w;
//
//		// object points corresponding to the edge points;
//		//std::vector<Point> points_object;
//		//std::vector<Pixel> pixels_object;
//
//		// for debugging
//		int index;
//	};
	
//	struct PointPair
//	{
//		Point *ref;
//		Point *cur;
//	};
//
	struct PlanePair
	{
		PlanePair(){}

		PlanePair(Plane *r, Plane *c)
		{
			ref=r;
			cur=c;
		}

		Plane *ref;
		Plane *cur;
	};

}
