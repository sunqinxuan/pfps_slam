/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-18 15:14
#
# Filename:		main.cpp
#
# Description: 
#
===============================================*/

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/Eigen/StdVector>
#include "data_reading.h"
#include "plane_feature_extraction.h"
#include "plane_feature_matching.h"
#include "pose_estimation.h"
#include "plane_map_update.h"
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <limits>
#include "display.h"

using namespace ulysses;


int main (int argc, char *argv[])
{
//	double a;
//	a=std::numeric_limits<double>::max();
//	std::cout<<pcl_isfinite(a)<<std::endl;
//	return 0;

	// some default settings;
	std::string sequence_name="/home/sun/dataset/rgbd_dataset_freiburg1_room";
//	int bins_d=1,bins_theta=10,bins_phy=20;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::NormalEstimationMethod method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::COVARIANCE_MATRIX;
	double MaxDepthChangeFactor=0.02f, NormalSmoothingSize=10.0f;
	double time_start=0;
	double time_interval=0.3;
	bool dbg_pfe=false, dbg_pfm=false, dbg_pe=false, dbg_pmu=false;
	double min_inliers=5000, ang_thres=0.017453*5.0, dist_thres=0.1, max_curv=0.01;
	float th_dd = 0.04f;
    int max_search = 100;
	int max_icp=10, max_lm=10;

	for(int i=1;i<argc;i++)
	{
		if(strcmp(argv[i],"-h")==0)
		{
			std::cout<<"-debug\tif debugging;"<<std::endl
					 <<"-ds\tdataset sequence name = /home/sun/dataset/rgbd_dataset_freiburg1_room;"<<std::endl
					 <<"-ne\tintegral image normal estimation method (method=1, max_depth_change=0.02, smoothing_size=10.0)  (method - 1:COVARIANCE_MATRIX, 2:AVERAGE_3D_GRADIENT, 3:AVERAGE_DEPTH_CHANGE)"<<std::endl
					 <<"-st\tstart time = 0;"<<std::endl
					 <<"-ti\ttime interval = 0.5;"<<std::endl;
			return 0;
		}
		if(strcmp(argv[i],"-dbg_pfe")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				dbg_pfe=true;
			if(strcmp(argv[i+1],"0")==0)
				dbg_pfe=false;
		}
		if(strcmp(argv[i],"-dbg_pfm")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				dbg_pfm=true;
			if(strcmp(argv[i+1],"0")==0)
				dbg_pfm=false;
		}
		if(strcmp(argv[i],"-dbg_pe")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				dbg_pe=true;
			if(strcmp(argv[i+1],"0")==0)
				dbg_pe=false;
		}
		if(strcmp(argv[i],"-dbg_pmu")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				dbg_pmu=true;
			if(strcmp(argv[i+1],"0")==0)
				dbg_pmu=false;
		}
		if(strcmp(argv[i],"-ds")==0)
		{
			sequence_name=argv[i+1];
		}
		if(strcmp(argv[i],"-ne")==0)
		{
			if(strcmp(argv[i+1],"COVARIANCE_MATRIX"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::COVARIANCE_MATRIX;
			}
			else if(strcmp(argv[i+1],"AVERAGE_3D_GRADIENT"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::AVERAGE_3D_GRADIENT;
			}
			else if(strcmp(argv[i+1],"AVERAGE_DEPTH_CHANGE"))
			{
				method=pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::AVERAGE_DEPTH_CHANGE;
			}
			MaxDepthChangeFactor=atof(argv[i+2]);
			NormalSmoothingSize=atof(argv[i+3]);
		}
		if(strcmp(argv[i],"-st")==0)
		{
			time_start=atof(argv[i+1]);
		}
		if(strcmp(argv[i],"-ti")==0)
		{
			time_interval=atof(argv[i+1]);
		}

		if(strcmp(argv[i],"-mi")==0)
		{
			min_inliers=atof(argv[i+1]);
		}
		if(strcmp(argv[i],"-at")==0)
		{
			ang_thres=atof(argv[i+1]);
			ang_thres*=0.017453;
		}
		if(strcmp(argv[i],"-dt")==0)
		{
			dist_thres=atof(argv[i+1]);
		}
		if(strcmp(argv[i],"-mc")==0)
		{
			max_curv=atof(argv[i+1]);
		}
		if(strcmp(argv[i],"-maxicp")==0)
		{
			max_icp=atoi(argv[i+1]);
		}
		if(strcmp(argv[i],"-maxlm")==0)
		{
			max_lm=atoi(argv[i+1]);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_scan_global(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// definitions for debugging;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointXYZRGBA tmp_point;

	DataReading *data_reading=new ulysses::DataReading(sequence_name);
	data_reading->setDebug(true);
	data_reading->Initialize(time_start);
	data_reading->setSampleInterval(time_interval);
	data_reading->setNormalEstimation(method, MaxDepthChangeFactor, NormalSmoothingSize);
	
	ulysses::Scan *scan_ref;
	ulysses::Scan *scan_cur;
	ulysses::IntrinsicParam int_param;

	// test pcl organized multi plane segmentation;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->addCoordinateSystem (1.0);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get ());

	ulysses::PlaneFeatureExtraction ops;
	ops.setMinInliers(min_inliers);
	ops.setAngularThreshold(ang_thres); // 10deg
	ops.setDistanceThreshold(dist_thres); // 10cm 
	ops.setMaximumCurvature(max_curv);
	ops.setDepthDisconThreshold (th_dd);
	ops.setMaxSearchNeighbors (max_search);
	ops.setEdgeType (ops.EDGELABEL_NAN_BOUNDARY | ops.EDGELABEL_OCCLUDING | ops.EDGELABEL_OCCLUDED);// | ops.EDGELABEL_HIGH_CURVATURE);
	ops.setDebug(dbg_pfe);

	ulysses::PlaneFeatureMatching pfm;
	pfm.setDebug(dbg_pfm);
	std::vector<ulysses::PlanePair> matched_planes;

	ulysses::PoseEstimation pe;
	pe.setDebug(dbg_pe);
	pe.setMaxIterationICP(max_icp);
	pe.setMaxIterationLM(max_lm);
	Transform Tcr_align_planes;
	Transform Tcr;
	Tcr.setIdentity();

	ulysses::PlaneMapUpdate pmu;
	pmu.setDebug(dbg_pmu);
	ulysses::Map *map=new ulysses::Map;

//	ulysses::ProjectiveRayAlignment pra;
//	pra.setDepthDisconThreshold (th_dd);
//	pra.setMaxSearchNeighbors (max_search);
//	pra.setEdgeType (pra.EDGELABEL_NAN_BOUNDARY | pra.EDGELABEL_OCCLUDING | pra.EDGELABEL_OCCLUDED);

//	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
//	std::vector<pcl::ModelCoefficients> model_coefficients;
//	std::vector<pcl::PointIndices> inlier_indices;  
//	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
//	std::vector<pcl::PointIndices> label_indices;
//	pcl::PointCloud<pcl::Label>::Ptr labels1 (new pcl::PointCloud<pcl::Label>);
//	std::vector<pcl::PointIndices> label_indices1;
//	std::vector<pcl::PointIndices> boundary_indices;

	std::vector<Plane*> planes;

	timeval start, end;
	char ch;
	double timeused;
	int first=0;
	int filenum = first;
	char cloud_id[20];
	ofstream fp;
	fp.open("traj.txt",std::ios::out);
//	CameraPose pose_tmp;
//	while(filenum<10)
	while(!data_reading->isEOF())
	{
		std::cout<<std::endl<<"***************** frame "<<filenum<<" ******************"<<std::endl;

		// load point cloud from the image sequence;
		gettimeofday(&start,NULL);
		scan_cur=data_reading->loadScan();
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		std::cout<<"time reading data from freiburg:"<<timeused<<std::endl;

		// observation extraction;
		gettimeofday(&start,NULL);
		ops.generateObservation(scan_cur);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		std::cout<<"time generating observations:"<<timeused<<std::endl;

		if(filenum==first)
		{
			// plane feature extraction;
			gettimeofday(&start,NULL);
			ops.generatePlaneFeature(scan_cur);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time generating plane feature:"<<timeused<<std::endl;

			// plane map update;
			gettimeofday(&start,NULL);
//			pmu.clearMap(map);
			pmu.updateMap(map,scan_cur);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time updating map:"<<timeused<<std::endl;

//			displayScan(scan_cur,vis);
//			vis->spin();
//			displayScanPlanes(scan_cur,vis);
//			vis->spin();
			display(scan_cur,filenum,vis);
			vis->spin();
		}




		if(filenum>first)
		{
			// plane feature extraction;
			gettimeofday(&start,NULL);
			ops.generatePlaneFeature(scan_cur);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time generating plane feature:"<<timeused<<std::endl;

//			// plane feature matching;
//			gettimeofday(&start,NULL);
////			pfm.match(map->planes, scan_cur->observed_planes, matched_planes);
//			pfm.match(scan_ref->observed_planes, scan_cur->observed_planes, matched_planes);
//			gettimeofday(&end,NULL);
//			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//			std::cout<<"time matching plane feature:"<<timeused<<std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////
//				// scan_cur and map;
//				std::cout<<"scan --- "<<std::endl;
//				for(size_t i=0;i<scan_cur->observed_planes.size();i++)
//				{
//					std::cout<<"plane "<<scan_cur->observed_planes[i]->index<<": "<<scan_cur->observed_planes[i]->count<<", "<<scan_cur->observed_planes[i]->coefficients.transpose()<<", "<<scan_cur->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
//				}
//				std::cout<<"map --- "<<std::endl;
//				for(size_t i=0;i<map->planes.size();i++)
//				{
//					std::cout<<"plane "<<map->planes[i]->index<<": "<<map->planes[i]->count<<", "<<map->planes[i]->coefficients.transpose()<<", "<<map->planes[i]->centroid_xyz.transpose()<<std::endl;
//					for(size_t j=0;j<scan_cur->observed_planes.size();j++)
//					{
//						std::cout<<"\t"<<map->planes[i]->similarity_geom(scan_cur->observed_planes[j])<<std::endl;
//					}
////					for(size_t j=0;j<map->planes[i]->ptr_camera_pose_seq.size();j++)
////						std::cout<<"\t"<<map->planes[i]->ptr_camera_pose_seq[j]->t.transpose()<<std::endl;
//				}
////				for(size_t i=0;i<map->camera_poses.size();i++)
////				{
////					std::cout<<"pose "<<i<<": "<<map->camera_poses[i]->t.transpose()<<std::endl;
////					for(size_t j=0;j<map->camera_poses[i]->ptr_planes.size();j++)
////						std::cout<<"\t"<<map->camera_poses[i]->ptr_planes[j]->coefficients.transpose()<<std::endl;
////				}
//////////////////////////////////////////////////////////////////////////////////////////

//				std::cout<<"scan_cur --- "<<std::endl;
//				for(size_t i=0;i<scan_cur->observed_planes.size();i++)
//				{
//					std::cout<<"plane "<<scan_cur->observed_planes[i]->index<<": "<<scan_cur->observed_planes[i]->count<<", "<<scan_cur->observed_planes[i]->coefficients.transpose()<<", "<<scan_cur->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
//				}
//				std::cout<<"scan_ref --- "<<std::endl;
//				for(size_t i=0;i<scan_ref->observed_planes.size();i++)
//				{
//					std::cout<<"plane "<<scan_ref->observed_planes[i]->index<<": "<<scan_ref->observed_planes[i]->count<<", "<<scan_ref->observed_planes[i]->coefficients.transpose()<<", "<<scan_ref->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
//					for(size_t j=0;j<scan_cur->observed_planes.size();j++)
//					{
//						std::cout<<"\t"<<scan_ref->observed_planes[i]->similarity_geom(scan_cur->observed_planes[j])<<std::endl;
//					}
//				}
//				std::cout<<"matched planes --- "<<std::endl;
//				for(size_t i=0;i<matched_planes.size();i++)
//				{
//					std::cout<<"<"<<matched_planes[i].cur->index<<","<<matched_planes[i].ref->index<<">"<<std::endl;
//				}


//			displayScanPlanes(scan_cur,vis);
//			vis->spin();

			gettimeofday(&start,NULL);
			//Tcr=*scan_ref->Tcg;
			Tcr.setIdentity();
			pe.align(scan_cur,scan_ref,Tcr);
//			pe.align(scan_cur,scan_ref,Tcr,vis);
//			pe.align(scan_cur,matched_planes,Tcr,vis);
//			Tcr.setIdentity();
//			pe.align(scan_cur,map,Tcr,vis);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time aligning planes:"<<timeused<<std::endl;
			std::cout<<"Tcr --- "<<std::endl<<Tcr.getMatrix4f()<<std::endl;




/////////////////////////////////////////////////////////////////////////////
/*
				Transform Tgc;
				Eigen::Matrix4f trans;
				vis->removeAllPointClouds();
				// ref;
				cloud_show_tmp->resize(scan_ref->point_cloud->size());
				for(size_t i=0;i<scan_ref->point_cloud->size();i++)
				{
					cloud_show_tmp->at(i)=scan_ref->point_cloud->at(i);
					cloud_show_tmp->at(i).r=0;
					cloud_show_tmp->at(i).g=0;
					cloud_show_tmp->at(i).b=255;
				}
				Tgc=scan_ref->Tcg->inv();
				trans=Tgc.getMatrix4f();
				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
				if (!vis->updatePointCloud (cloud_show_tmp, "ref"))
					vis->addPointCloud (cloud_show_tmp,"ref");
				// cur;
				cloud_show_tmp->resize(scan_cur->point_cloud->size());
				for(size_t i=0;i<scan_cur->point_cloud->size();i++)
				{
					cloud_show_tmp->at(i)=scan_cur->point_cloud->at(i);
					cloud_show_tmp->at(i).r=255;
					cloud_show_tmp->at(i).g=0;
					cloud_show_tmp->at(i).b=0;
				}
				Tgc=scan_ref->Tcg->inv();
				trans=Tgc.getMatrix4f();
				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
				if (!vis->updatePointCloud (cloud_show_tmp, "cur"))
					vis->addPointCloud (cloud_show_tmp,"cur");
				// occluding_points_seq[0] from map;
				cloud_show_tmp->resize(0);
				for(size_t i=0;i<map->planes.size();i++)
				{
					size_t N=cloud_show_tmp->size();
					cloud_show_tmp->resize(N+map->planes[i]->occluding_points_seq[0].size());
					for(size_t j=0;j<map->planes[i]->occluding_points_seq[0].size();j++)
					{
						cloud_show_tmp->at(N+j).x=map->planes[i]->occluding_points_seq[0][j]->xyz(0);
						cloud_show_tmp->at(N+j).y=map->planes[i]->occluding_points_seq[0][j]->xyz(1);
						cloud_show_tmp->at(N+j).z=map->planes[i]->occluding_points_seq[0][j]->xyz(2);
						cloud_show_tmp->at(N+j).r=0;
						cloud_show_tmp->at(N+j).g=255;
						cloud_show_tmp->at(N+j).b=0;
					}
				}
				if (!vis->updatePointCloud (cloud_show_tmp, "real_contour_map"))
					vis->addPointCloud (cloud_show_tmp,"real_contour_map");
				vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "real_contour_map");
				// occluding_points_seq[0] from cur_scan;
				cloud_show_tmp->resize(0);
				for(size_t i=0;i<scan_cur->observed_planes.size();i++)
				{
					size_t N=cloud_show_tmp->size();
					cloud_show_tmp->resize(N+scan_cur->observed_planes[i]->occluding_points_seq[0].size());
					for(size_t j=0;j<scan_cur->observed_planes[i]->occluding_points_seq[0].size();j++)
					{
						cloud_show_tmp->at(N+j).x=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(0);
						cloud_show_tmp->at(N+j).y=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(1);
						cloud_show_tmp->at(N+j).z=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(2);
						cloud_show_tmp->at(N+j).r=255;
						cloud_show_tmp->at(N+j).g=255;
						cloud_show_tmp->at(N+j).b=0;
					}
				}
				Tgc=scan_ref->Tcg->inv();
				trans=Tgc.getMatrix4f();
				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
				if (!vis->updatePointCloud (cloud_show_tmp, "real_contour_scan"))
					vis->addPointCloud (cloud_show_tmp,"real_contour_scan");
				vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "real_contour_scan");
				vis->spin();

				// cur after trans;
				cloud_show_tmp->resize(scan_cur->point_cloud->size());
				for(size_t i=0;i<scan_cur->point_cloud->size();i++)
				{
					cloud_show_tmp->at(i)=scan_cur->point_cloud->at(i);
					cloud_show_tmp->at(i).r=255;
					cloud_show_tmp->at(i).g=0;
					cloud_show_tmp->at(i).b=0;
				}
				Tgc=scan_cur->Tcg->inv();
				trans=Tgc.getMatrix4f();
				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
//					sprintf(cloud_id,"cloud%d",filenum);
//					if(!vis->updatePointCloud (cloud_show_tmp, cloud_id))
//						vis->addPointCloud(cloud_show_tmp, cloud_id);
				if (!vis->updatePointCloud (cloud_show_tmp, "cur"))
					vis->addPointCloud (cloud_show_tmp,"cur");
				// occluding_points_seq[0] after trans;
				cloud_show_tmp->resize(0);
				for(size_t i=0;i<scan_cur->observed_planes.size();i++)
				{
					size_t N=cloud_show_tmp->size();
					cloud_show_tmp->resize(N+scan_cur->observed_planes[i]->occluding_points_seq[0].size());
					for(size_t j=0;j<scan_cur->observed_planes[i]->occluding_points_seq[0].size();j++)
					{
						cloud_show_tmp->at(N+j).x=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(0);
						cloud_show_tmp->at(N+j).y=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(1);
						cloud_show_tmp->at(N+j).z=scan_cur->observed_planes[i]->occluding_points_seq[0][j]->xyz(2);
						cloud_show_tmp->at(N+j).r=255;
						cloud_show_tmp->at(N+j).g=255;
						cloud_show_tmp->at(N+j).b=0;
					}
				}
				Tgc=scan_cur->Tcg->inv();
				trans=Tgc.getMatrix4f();
				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
				if (!vis->updatePointCloud (cloud_show_tmp, "real_contour_scan"))
					vis->addPointCloud (cloud_show_tmp,"real_contour_scan");
				vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "real_contour_scan");
				vis->spin();
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			// plane map update;
			gettimeofday(&start,NULL);
//			pmu.clearMap(map);
			pmu.updateMap(map,scan_cur);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time updating map:"<<timeused<<std::endl;

//			displayMapPlanePoints(map,vis);
//			if(filenum%10==0)
//				vis->spin();

			display(scan_cur,filenum,vis);
			vis->spin();

			delete scan_ref;
		}

//			displayPlanarRegions(scan_cur->planar_regions, vis, scan_cur->point_cloud, scan_cur->plane_indices);
//			vis->spin();

//			displayEdges(scan_cur->edge_indices,vis,scan_cur->point_cloud);
//			if (!vis->updatePointCloud (scan_cur->point_cloud, "cur"))
//				vis->addPointCloud (scan_cur->point_cloud,"cur");
//			vis->spin();
//
//
//			// projective ray alignment;
//			gettimeofday(&start,NULL);
//			Tcr = Eigen::Isometry3d::Identity();
//			pra.align(scan_ref,scan_cur,Tcr,int_param,vis);
//			gettimeofday(&end,NULL);
//			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//			std::cout<<"time aligning:"<<timeused<<std::endl;
//			std::cout<<"Tcr="<<std::endl<<Tcr.matrix() <<std::endl;
//
//			Tcg=Tcr*Tcg;
//			pose_tmp.Rotation = Tcg.inverse().rotation();
//			pose_tmp.position = Tcg.inverse().translation();
//			std::cout<<"getTime: "<<data_reading->getTime()<<std::endl;
//			fp<<std::fixed<<data_reading->getTime()<<" "<<pose_tmp.position.transpose()<<" "<<pose_tmp.Quat().vec().transpose()<<" "<<pose_tmp.Quat().w()<<std::endl;
//
//			Eigen::Matrix4f trans;
//			for(size_t i=0;i<4;i++)
//			{
//				for(size_t j=0;j<4;j++)
//				{
//					trans(i,j)=Tcr.matrix()(i,j);
//				}
//			}


//			std::cout<<scan_ref<<","<<scan_cur<<std::endl;
//			std::cout<<scan_ref->point_cloud<<","<<scan_cur->point_cloud<<std::endl;
			

//			// show the two point clouds before transforming;
//			for(size_t i=0;i<scan_ref->point_cloud->size();i++)
//			{
//				scan_ref->point_cloud->at(i).r=255;
//				scan_ref->point_cloud->at(i).g=0;
//				scan_ref->point_cloud->at(i).b=0;
//			}
//			for(size_t i=0;i<scan_cur->point_cloud->size();i++)
//			{
//				scan_cur->point_cloud->at(i).r=0;
//				scan_cur->point_cloud->at(i).g=0;
//				scan_cur->point_cloud->at(i).b=255;
//			}
//			cloud_show->clear();
//			*cloud_show=*scan_ref->point_cloud+*scan_cur->point_cloud;
//			if (!vis->updatePointCloud (cloud_show, cloud_id)) //<pcl::PointXYZRGBA>
//				vis->addPointCloud (cloud_show,cloud_id);// "sample cloud");
//			vis->spin();
//
//			// show the two point clouds after transforming;
//			vis->removeAllPointClouds();
//			transformPointCloud(*scan_ref->point_cloud,*scan_ref->point_cloud,trans);
//			if (!vis->updatePointCloud (scan_ref->point_cloud, "r")) //<pcl::PointXYZRGBA>
//				vis->addPointCloud (scan_ref->point_cloud,"r");// "sample cloud");
//			if (!vis->updatePointCloud (scan_cur->point_cloud, "ii")) //<pcl::PointXYZRGBA>
//				vis->addPointCloud (scan_cur->point_cloud,"ii");// "sample cloud");
//			vis->spin();
			

//			if(debug)
//			{
			/*	
				// show the edge points before transforming;
				cloud_show->clear();
				vis->removeAllPointClouds();
				for(size_t i=0;i<scan_ref->label_indices[1].indices.size();i++)
				{
					int index=scan_ref->label_indices[1].indices[i];
					tmp_point=scan_ref->point_cloud->at(index);
					tmp_point.r=255;
					tmp_point.g=0;
					tmp_point.b=0;
					cloud_show->push_back(tmp_point);
				}getTransform
				for(size_t i=0;i<scan_cur->label_indices[1].indices.size();i++)
				{
					int index=scan_cur->label_indices[1].indices[i];
					tmp_point=scan_cur->point_cloud->at(index);
					tmp_point.r=0;
					tmp_point.g=0;
					tmp_point.b=255;
					cloud_show->push_back(tmp_point);	ulysses::ProjectiveRayAlignment pra;
	pra.setDepthDisconThreshold (th_dd);
	pra.setMaxSearchNeighbors (max_search);
	pra.setEdgeType (pra.EDGELABEL_NAN_BOUNDARY | pra.EDGELABEL_OCCLUDING | pra.EDGELABEL_OCCLUDED);
				}
				if (!vis->updatePointCloud (cloud_show, cloud_id)) //<pcl::PointXYZRGBA>
					vis->addPointCloud (cloud_show,cloud_id);// "sample cloud");
				vis->spin();

				// show the edge points after transforming;
				cloud_show->clear();
				vis->removeAllPointClouds();
				for(size_t i=0;i<scan_ref->label_indices[1].indices.size();i++)
				{
					int index=scan_ref->label_indices[1].indices[i];
					tmp_point=scan_ref->point_cloud->at(index);
					tmp_point.r=255;
					tmp_point.g=0;
					tmp_point.b=0;
					cloud_show->push_back(tmp_point);
				}
				transformPointCloud(*cloud_show,*cloud_show,trans);
				for(size_t i=0;i<scan_cur->label_indices[1].indices.size();i++)
				{
					int index=scan_cur->label_indices[1].indices[i];
					tmp_point=scan_cur->point_cloud->at(index);
					tmp_point.r=0;
					tmp_point.g=0;
					tmp_point.b=255;
					cloud_show->push_back(tmp_point);
				}
				if (!vis->updatePointCloud (cloud_show, cloud_id)) //<pcl::PointXYZRGBA>
					vis->addPointCloud (cloud_show,cloud_id);// "sample cloud");
				vis->spin();
				*/

//				*cloud_show_tmp=*scan_cur->point_cloud;
//				transformPointCloud(*cloud_show_tmp,*cloud_show_tmp,trans);
//				*cloud_show=*cloud_show+*cloud_show_tmp;
//				if (!vis->updatePointCloud (cloud_show, cloud_id)) //<pcl::PointXYZRGBA>
//					vis->addPointCloud (cloud_show,cloud_id);// "sample cloud");
//				vis->spin();
//			}



//			std::cout<<"regions:"<<regions.size()<<std::endl;
//			for(size_t i=0;i<regions.size();i++)
//			{
//				Eigen::Vector3f centroid=regions[i].getCentroid();
//				Eigen::Vector4f model=regions[i].getCoefficients();
//				pcl::PointCloud<pcl::PointXYZRGBA> boundary_cloud;
//				boundary_cloud.points=regions[i].getContour();
//				std::cout<<std::endl<<"Centroid:"<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<std::endl
//					<<"Coefficients:"<<model[0]<<" "<<model[1]<<" "<<model[2]<<" "<<model[3]<<std::endl
//					<<"boundary point size:"<<boundary_cloud.points.size()<<std::endl<<std::endl;
//			}
//			if (!vis->updatePointCloud (scan_cur->point_cloud, cloud_id)) //<pcl::PointXYZRGBA>
//				vis->addPointCloud (scan_cur->point_cloud,cloud_id);// "sample cloud");
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id);

			
//			ofstream fp_label;
//			fp_label.open("labels.txt",std::ios::out);
//			for(size_t i=0;i<scan_cur->edge_label_cloud->height;i++)
//			{
//				for(size_t j=0;j<scan_cur->edge_label_cloud->width;j++)
//				{
//					fp_label<<scan_cur->edge_label_cloud->at(j,i).label<<" ";
//				}
//				fp_label<<std::endl;
//			}
//			fp_label.close();
//
//			ofstream fp_label_seg;
//			fp_label_seg.open("labels_seg.txt",std::ios::out);
//			for(size_t i=0;i<scan_cur->plane_label_cloud->height;i++)
//			{
//				for(size_t j=0;j<scan_cur->plane_label_cloud->width;j++)
//				{
//					fp_label_seg<<scan_cur->plane_label_cloud->at(j,i).label<<" ";
//				}
//				fp_label_seg<<std::endl;
//			}
//			fp_label_seg.close();
//
//			std::cout<<"planes:"<<scan_cur->plane_indices.size()<<std::endl;


			// show the edge detection results;


//			std::cout<<"edges:"<<scan_cur->edge_indices.size()<<std::endl;
//			for(size_t i=0;i<scan_cur->edge_indices.size();i++)
//			{
//				std::cout<<i<<"-th: "<<scan_cur->edge_indices[i].indices.size()<<std::endl;
//			}

//
//			char id[20];
//			for(int i=0;i<scan_cur->edge_indices[1].indices.size();i++)
//			{
//				int idx1=scan_cur->edge_indices[1].indices[i];
//				int idx2=scan_cur->edge_indices[2].indices[scan_cur->occluding_correspondences.indices[i]];
//				if(scan_cur->occluding_correspondences.indices[i]==-1)
//					continue;
//				pcl::PointXYZRGBA point;
//				point.x=scan_cur->point_cloud->at(idx1).x+scan_cur->projective_rays[i](0);
//				point.y=scan_cur->point_cloud->at(idx1).y+scan_cur->projective_rays[i](1);
//				point.z=scan_cur->point_cloud->at(idx1).z+scan_cur->projective_rays[i](2);
//				sprintf(id,"%d",i);
//				vis->removeShape(id);
////				vis->addLine<pcl::PointXYZRGBA>(scan_cur->point_cloud->at(idx1),scan_cur->point_cloud->at(idx2),0,1,0,id);
//				vis->addLine<pcl::PointXYZRGBA>(scan_cur->point_cloud->at(idx1),point,0,1,0,id);
//			}
//
//			
//			displayPointsOnPlane(scan_cur->planar_regions,vis,scan_cur->point_cloud, scan_cur->edge_indices[1], scan_cur->edge_corresponding_plane[1]);

	//		std::cout<<"labels:"<<scan_ref->label_cloud->size()<<std::endl;
	//		std::cout<<"labels size: "<<scan_ref-//				if(dist[i]>0.1)
//				{
//					continue;
//				}
//
//				sprintf(id,"%d%d",iterations,i);
//				vis->removeShape(id);
////				if(dist[i]>0.01)
//				vis->addLine<pcl::PointXYZRGBA>(tmp_point,scan_cur->point_cloud->at(index_cur),0,1,0,id);>label_cloud->height<<","<<scan_ref->label_cloud->width<<std::endl;
//
//			cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
//			for (size_t i=0;i<scan_cur->edge_label_cloud->height;i++)
//			{
//				for (size_t j=0;j<scan_cur->edge_label_cloud->width;j++)
//				{
//					if(scan_cur->edge_label_cloud->at(j,i).label==1)
//					{
//						plane_img.at<cv::Vec3b>(i,j)[0]=255;
//						plane_img.at<cv::Vec3b>(i,j)[1]=0;
//						plane_img.at<cv::Vec3b>(i,j)[2]=0;
//					}
//					if(scan_cur->edge_label_cloud->at(j,i).label==2)
//					{
//						plane_img.at<cv::Vec3b>(i,j)[0]=0;
//						plane_img.at<cv::Vec3b>(i,j)[1]=255;
//						plane_img.at<cv::Vec3b>(i,j)[2]=0;
//					}
//					if(scan_cur->edge_label_cloud->at(j,i).label==4)
//					{
//						plane_img.at<cv::Vec3b>(i,j)[0]=0;
//						plane_img.at<cv::Vec3b>(i,j)[1]=0;
//						plane_img.at<cv::Vec3b>(i,j)[2]=255;
//					}
//				}
//			}
//			for(size_t i=0;i<scan_cur->occluding_correspondences.indices.size();i++)
//			{
//				cv::Point pt1,pt2;
////				scan_cur->edge_indices[1].indices[i]
//				int idx1=scan_cur->edge_indices[1].indices[i];
//				int idx2=scan_cur->edge_indices[2].indices[scan_cur->occluding_correspondences.indices[i]];
//				if(scan_cur->occluding_correspondences.indices[i]==-1)
//					continue;
//				pt1.y=scan_cur->pixel_cloud->at(idx1).x;
//				pt1.x=scan_cur->pixel_cloud->at(idx1).y;
//				pt2.y=scan_cur->pixel_cloud->at(idx2).x;
//				pt2.x=scan_cur->pixel_cloud->at(idx2).y;
//				line(plane_img,pt1,pt2,CV_RGB(255,255,255));
//				fp<<pt1.x<<" "<<pt1.y<<" "<<pt2.x<<" "<<pt2.y<<std::endl;
//			}
//			cv::imshow("img",plane_img);
//			cv::waitKey(0);


//		viewer.showCloud(scan_cur->point_cloud);
//		std::cout<<"Press enter to continue...\n"<<std::endl;
//		ch=std::cin.get();

	
		scan_ref=scan_cur;
		filenum++;

//		vis->spinOnce();
	}
//			vis->spin();
	
	fp.close();
	return 0;
}
