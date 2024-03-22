/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-06-12 17:10
#
# Filename: plane_map_update.cpp
#
# Description: 
#
===============================================*/

#include "plane_map_update.h"

namespace ulysses
{
	void PlaneMapUpdate::clearMap(Map *map)
	{
		for(int i=0;i<map->planes.size();i++)
			delete map->planes[i];
		for(int i=0;i<map->camera_poses.size();i++)
			delete map->camera_poses[i];
	}

	void PlaneMapUpdate::updateMap(Map *map, Scan *scan)//, std::vector<PlanePair> matched_planes)//, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		// add the camera pose of current scan into the map;
		map->camera_poses.push_back(scan->Tcg_gt);
		if(map->planes.size()==0)
		{
			// when the map is empty;
			// push the observed_planes in first scan into map;
			map->planes.resize(scan->observed_planes.size());
			for(size_t i=0;i<map->planes.size();i++)
			{
				map->planes[i]=scan->observed_planes[i];
				map->camera_poses[0]->ptr_planes.push_back(map->planes[i]);
			}
		}
		else
		{
			Transform Tgc=scan->Tcg_gt->inv();
			std::vector<SalientPoint*> points;

			if(debug)
				std::cout<<"planes in scan (transformed into global frame) --- "<<scan->observed_planes.size()<<std::endl;

			for(size_t i=0;i<scan->observed_planes.size();i++)
			{
				// for each observed palne; 
				// project scan->observed_planes[i] into the global frame;
				scan->observed_planes[i]->coefficients=Tgc.transformPlane(scan->observed_planes[i]->coefficients);
				scan->observed_planes[i]->centroid_xyz=Tgc.transform(scan->observed_planes[i]->centroid_xyz);
				scan->observed_planes[i]->covariance_xyz=Tgc.R*scan->observed_planes[i]->covariance_xyz*Tgc.R.transpose();
				for(size_t j=0;j<scan->observed_planes[i]->contour.size();j++)
				{
					scan->observed_planes[i]->contour[j]=Tgc.transform(scan->observed_planes[i]->contour[j]);
				}
				points.clear();
				points.insert(points.end(),scan->observed_planes[i]->occluding_points_seq[0].begin(),
										   scan->observed_planes[i]->occluding_points_seq[0].end());
				points.insert(points.end(),scan->observed_planes[i]->occluded_points_seq[0].begin(),
										   scan->observed_planes[i]->occluded_points_seq[0].end());
				for(size_t j=0;j<points.size();j++)
				{
					Tgc.transform(points[j]);
				}

				Eigen::Matrix4f trans=Tgc.getMatrix4f();
				transformPointCloud(*scan->observed_planes[i]->points,*scan->observed_planes[i]->points,trans);

				if(debug)
				{
					std::cout<<i<<"th plane in scan ---"<<std::endl;
					std::cout<<"\tcoefficients: "<<scan->observed_planes[i]->coefficients.transpose()<<std::endl;
					std::cout<<"\tcentroid_xyz: "<<scan->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
					std::cout<<"\tcovariance_xyz: "<<std::endl<<scan->observed_planes[i]->covariance_xyz<<std::endl;
					std::cout<<"\tcentroid_rgb: "<<scan->observed_planes[i]->centroid_rgb.transpose()<<std::endl;
					std::cout<<"\tcount: "<<scan->observed_planes[i]->count<<std::endl;
					std::cout<<"\tcurvature: "<<scan->observed_planes[i]->curvature<<std::endl;
				}
				
				Plane *plane;
				double sim, min=9999999;
				for(size_t j=0;j<map->planes.size();j++)
				{ // for each plane feature in the map;
					sim=scan->observed_planes[i]->similarity_angle(map->planes[j])+scan->observed_planes[i]->similarity_color(map->planes[j])+scan->observed_planes[i]->similarity_dist(map->planes[j]);
					sim+=scan->observed_planes[i]->distancePoint2Plane(map->planes[j]);
					Eigen::Vector3d centroid=scan->observed_planes[i]->centroid_xyz-map->planes[j]->centroid_xyz;
					sim+=centroid.norm();
					if(debug)
					{
						std::cout<<"scan-"<<i<<" map-"<<j<<" ---"<<std::endl;
						std::cout<<"\tangle: "<<scan->observed_planes[i]->similarity_angle(map->planes[j])<<std::endl;
//						if(i==2 && j==3)
//						{
//							double tmp=scan->observed_planes[i]->coefficients.block<3,1>(0,0).transpose()*map->planes[j]->coefficients.block<3,1>(0,0);
//							double s_ang=acos(tmp);
//							std::cout<<"======================"<<tmp<<","<<s_ang<<std::endl;
//						}
						std::cout<<"\tcolor: "<<scan->observed_planes[i]->similarity_color(map->planes[j])<<std::endl;
						std::cout<<"\tdist: "<<scan->observed_planes[i]->similarity_dist(map->planes[j])<<std::endl;
						std::cout<<"\tPoint2Plane: "<<scan->observed_planes[i]->distancePoint2Plane(map->planes[j])<<std::endl;
						std::cout<<"\tcentroid: "<<centroid.norm()<<std::endl;
					}
					if(sim<min)
					{
						min=sim;
						plane=map->planes[j];
					}
					
//					std::cout<<"scan-"<<i<<" map-"<<j<<" --- ";
//					std::cout<<scan->observed_planes[i]->distancePoint2Plane(map->planes[j])<<std::endl;
				}

				if(debug)
				{
					std::cout<<"nearest plane in map ---"<<std::endl;
					std::cout<<"\tcoefficients: "<<plane->coefficients.transpose()<<std::endl;
					std::cout<<"\tcentroid_xyz: "<<plane->centroid_xyz.transpose()<<std::endl;
					std::cout<<"\tcovariance_xyz: "<<std::endl<<plane->covariance_xyz<<std::endl;
					std::cout<<"\tcentroid_rgb: "<<plane->centroid_rgb.transpose()<<std::endl;
					std::cout<<"\tcount: "<<plane->count<<std::endl;
					std::cout<<"\tcurvature: "<<plane->curvature<<std::endl;
				}

				// plane - most similar plane in map to scan->observed_planes[i];
				if(scan->observed_planes[i]->similar(plane,30,1,0.5))
				{
					if(debug)
						std::cout<<"merge plane ..."<<std::endl;

					// there exists a similar plane in map;
					// merge them;
					mergePlanes(plane,scan->observed_planes[i]);
//					plane->ptr_camera_pose_seq.push_back(scan->Tcg);
					bool flag=true;
					for(size_t j=0;j<map->camera_poses[map->camera_poses.size()-1]->ptr_planes.size();j++)
					{
						if(map->camera_poses[map->camera_poses.size()-1]->ptr_planes[j]==plane)
							flag=false;
					}
					if(flag)
						map->camera_poses[map->camera_poses.size()-1]->ptr_planes.push_back(plane);

					if(debug)
					{
						std::cout<<"after merging ---"<<std::endl;
						std::cout<<"\tcoefficients: "<<plane->coefficients.transpose()<<std::endl;
						std::cout<<"\tcentroid_xyz: "<<plane->centroid_xyz.transpose()<<std::endl;
						std::cout<<"\tcovariance_xyz: "<<std::endl<<plane->covariance_xyz<<std::endl;
						std::cout<<"\tcentroid_rgb: "<<plane->centroid_rgb.transpose()<<std::endl;
						std::cout<<"\tcount: "<<plane->count<<std::endl;
						std::cout<<"\tcurvature: "<<plane->curvature<<std::endl;
					}
				}
				else
				{
					if(debug)
						std::cout<<"add a new plane to map..."<<std::endl;
					// there isn't a similar plane in map;
					// add scan->observed_planes[i] as a new plane in the map;
					map->planes.push_back(scan->observed_planes[i]);
					map->camera_poses[map->camera_poses.size()-1]->ptr_planes.push_back(scan->observed_planes[i]);
				}
			}
		}
	}

	void PlaneMapUpdate::mergePlanes(Plane *plane, Plane *plane_scan)
	{
		*plane->points=*plane->points+*plane_scan->points;

		// centroid_xyz, covariance_xyz;
		Eigen::Vector3d avg_xyz_map=plane->centroid_xyz;
		Eigen::Vector3d avg_xyz_scan=plane_scan->centroid_xyz;
		plane->centroid_xyz=(plane->count*avg_xyz_map+plane_scan->count*avg_xyz_scan)/(plane->count+plane_scan->count);
		plane->covariance_xyz=(plane->count*(plane->covariance_xyz+avg_xyz_map*avg_xyz_map.transpose())+plane_scan->count*(plane_scan->covariance_xyz+avg_xyz_scan*avg_xyz_scan.transpose()))/(plane->count+plane_scan->count)-plane->centroid_xyz*plane->centroid_xyz.transpose();

		// centroid_rgb, covariance_rgb;
		Eigen::Vector3d avg_rgb_map=plane->centroid_rgb;
		Eigen::Vector3d avg_rgb_scan=plane_scan->centroid_rgb;
		plane->centroid_rgb=(plane->count*avg_rgb_map+plane_scan->count*avg_rgb_scan)/(plane->count+plane_scan->count);
		plane->covariance_rgb=(plane->count*(plane->covariance_rgb+avg_rgb_map*avg_rgb_map.transpose())+plane_scan->count*(plane_scan->covariance_rgb+avg_rgb_scan*avg_rgb_scan.transpose()))/(plane->count+plane_scan->count)-plane->centroid_rgb*plane->centroid_rgb.transpose();

		EIGEN_ALIGN16 Eigen::Vector3d eigen_value;
		EIGEN_ALIGN16 Eigen::Matrix3d eigen_vector;
		pcl::eigen33 (plane->covariance_xyz, eigen_vector, eigen_value);
		// coefficients;
		plane->coefficients.block<3,1>(0,0)=eigen_vector.col(0);//.block<3,1>(0,0);
		plane->coefficients(3)=-1*eigen_vector.col(0).dot(plane->centroid_xyz);
		if(plane->coefficients(3)<0)
			plane->coefficients=-plane->coefficients;
		else
			plane->coefficients=plane->coefficients;
		// curvature;
		double eig_sum = plane->covariance_xyz(0,0)+plane->covariance_xyz(1,1)+plane->covariance_xyz(2,2);
		if (eig_sum != 0)
			plane->curvature= fabsf (eigen_value(0) / eig_sum);
		else
			plane->curvature = -1;
		// count;
		plane->count+=plane_scan->count;
		// size;
		plane->size=sqrt(eigen_value(2));

		// camera pose;
		plane->ptr_camera_pose_seq.push_back(plane_scan->ptr_camera_pose_seq[0]);
		plane->occluding_points_seq.push_back(plane_scan->occluding_points_seq[0]);
		plane->occluded_points_seq.push_back(plane_scan->occluded_points_seq[0]);

//		delete plane_scan;

//		// real_contour;
//		ANNkd_tree *kdtree;
//		ANNpointArray kdtree_points;
//		kdtree_points=annAllocPts(plane->real_contour.size(),3);
//		for(size_t i=0;i<plane->real_contour.size();i++)
//		{
//			kdtree_points[i][0]=plane->real_contour[i](0);
//			kdtree_points[i][1]=plane->real_contour[i](1);
//			kdtree_points[i][2]=plane->real_contour[i](2);
//		}
//		kdtree=new ANNkd_tree(kdtree_points,plane->real_contour.size(),3);
//		ANNpoint query_point=annAllocPt(3);
//		ANNidxArray index=new ANNidx[1];
//		ANNdistArray distance=new ANNdist[1];
//		for(size_t i=0;i<plane_scan->real_contour.size();i++)
//		{
//			query_point[0]=plane_scan->real_contour[i](0);
//			query_point[1]=plane_scan->real_contour[i](1);
//			query_point[2]=plane_scan->real_contour[i](2);
//			kdtree->annkSearch(query_point,1,index,distance,0);
//			if(distance[0]<0.1)
//			{
//				plane->real_contour[index[0]]=0.5*(plane->real_contour[index[0]]+plane_scan->real_contour[i]);
//			}
//			else
//			{
//				plane->real_contour.push_back(plane_scan->real_contour[i]);
//			}
//		}


//		if(debug)
//			std::cout<<"merging occluding points ..."<<std::endl;
//
//		// merge occluding points;
//		int no_frm=plane->occluding_points_seq.size(); //no. of frames included in the map;
//		std::vector<SalientPoint*> spts; //all the points in map that is going to be associated;
//		if(no_frm>=win_size)
//		{
//			for(int i=no_frm-1;i>no_frm-1-win_size;i--)
//			{
//				spts.insert(spts.end(),plane->occluding_points_seq[i].begin(),plane->occluding_points_seq[i].end());
//			}
//		}
//		else
//		{
//			for(int i=no_frm-1;i>=0;i--)
//			{
//				spts.insert(spts.end(),plane->occluding_points_seq[i].begin(),plane->occluding_points_seq[i].end());
//			}
//		}
//		if(debug)
//			std::cout<<"spts: "<<spts.size()<<std::endl;
//		plane->occluding_points_seq.push_back(plane_scan->occluding_points_seq[0]);
//		if(debug)
//			std::cout<<"occluding points "<<plane->occluding_points_seq.size()<<std::endl;
//		// query_point;
//		// from plane_scan->occluding_points_seq[0];
//		ANNpoint query_point=annAllocPt(3);
//		ANNidxArray index=new ANNidx[1];
//		ANNdistArray distance=new ANNdist[1];
//		// kdtree_points - spts
//		// points that are used to build the kdtree;
//		ANNpointArray kdtree_points;
//		kdtree_points=annAllocPts(spts.size(),3);
//		for(size_t k=0;k<spts.size();k++)
//		{
//			kdtree_points[k][0]=spts[k]->xyz(0);
//			kdtree_points[k][1]=spts[k]->xyz(1);
//			kdtree_points[k][2]=spts[k]->xyz(2);
//		}
//		ANNkd_tree *kdtree=new ANNkd_tree(kdtree_points,spts.size(),3);
//		// for each point in plane_scan->occluding_points_seq[0];
//		// search spts for its nearest;
//		for(size_t k=0;k<plane_scan->occluding_points_seq[0].size();k++)
//		{
//			query_point[0]=plane_scan->occluding_points_seq[0][k]->xyz(0);
//			query_point[1]=plane_scan->occluding_points_seq[0][k]->xyz(1);
//			query_point[2]=plane_scan->occluding_points_seq[0][k]->xyz(2);
//			kdtree->annkSearch(query_point,1,index,distance,0);
//			if(distance[0]<0.01)
//				plane_scan->occluding_points_seq[0][k]->ptr_nn=spts[index[0]];
//		}
//		delete kdtree;
//		annDeallocPts(kdtree_points);
//
//		if(debug)
//			std::cout<<"merging occluded points ..."<<std::endl;
//
//		// merge occluded points;
//		spts.clear();
//		if(no_frm>=win_size)
//		{
//			for(int i=no_frm-1;i>no_frm-1-win_size;i--)
//			{
//				spts.insert(spts.end(),plane->occluded_points_seq[i].begin(),plane->occluded_points_seq[i].end());
//			}
//		}
//		else
//		{
//			for(int i=no_frm-1;i>=0;i--)
//			{
//				spts.insert(spts.end(),plane->occluded_points_seq[i].begin(),plane->occluded_points_seq[i].end());
//			}
//		}
//		plane->occluded_points_seq.push_back(plane_scan->occluded_points_seq[0]);
//		// kdtree_points - spts
//		// points that are used to build the kdtree;
//		kdtree_points=annAllocPts(spts.size(),3);
//		for(size_t k=0;k<spts.size();k++)
//		{
//			kdtree_points[k][0]=spts[k]->xyz(0)+spts[k]->proj_ray(0);
//			kdtree_points[k][1]=spts[k]->xyz(1)+spts[k]->proj_ray(1);
//			kdtree_points[k][2]=spts[k]->xyz(2)+spts[k]->proj_ray(2);
//		}
//		kdtree=new ANNkd_tree(kdtree_points,spts.size(),3);
//		// for each point in plane_scan->occluded_points_seq[0];
//		// search spts for its nearest;
//		for(size_t k=0;k<plane_scan->occluded_points_seq[0].size();k++)
//		{
//			query_point[0]=plane_scan->occluded_points_seq[0][k]->xyz(0)+plane_scan->occluded_points_seq[0][k]->proj_ray(0);
//			query_point[1]=plane_scan->occluded_points_seq[0][k]->xyz(1)+plane_scan->occluded_points_seq[0][k]->proj_ray(1);
//			query_point[2]=plane_scan->occluded_points_seq[0][k]->xyz(2)+plane_scan->occluded_points_seq[0][k]->proj_ray(2);
//			kdtree->annkSearch(query_point,1,index,distance,0);
//			if(distance[0]<0.01)
//				plane_scan->occluded_points_seq[0][k]->ptr_nn=spts[index[0]];
//		}
//		delete kdtree;
//		annDeallocPts(kdtree_points);
//
//		delete index;
//		delete distance;
//		annDeallocPt(query_point);
	}
}
