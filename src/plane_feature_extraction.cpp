/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-12 10:43
#
# Filename:		plane_feature_extraction.cpp
#
# Description: 
#
===============================================*/

#include "plane_feature_extraction.h"
#include <pcl-1.8/pcl/filters/extract_indices.h>

namespace ulysses
{
	void PlaneFeatureExtraction::initialize()
	{
		double min_inliers=5000, ang_thres=0.017453*5.0, dist_thres=0.05, max_curv=0.1;
		float th_dd = 0.04f;
		int max_search = 100;
		// plane segmentation;
		setMinInliers(min_inliers);
		setAngularThreshold(ang_thres); // 5deg
		setDistanceThreshold(dist_thres); // 5cm 
		setMaximumCurvature(max_curv);
		// edge detection;
		setDepthDisconThreshold (th_dd);
		setMaxSearchNeighbors (max_search);
		setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
	}


	void PlaneFeatureExtraction::segmentPlanes(Scan *scan)
	{
		std::vector<pcl::ModelCoefficients> model_coefficients;
		pcl::PointCloud<pcl::Label>::Ptr segment_label_cloud(new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> segment_indices;
		std::vector<pcl::PointIndices> segment_boundary_indices;

//		// segment
//		scan->segment_label_cloud=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
//		scan->segment_indices.clear();
//		scan->segment_boundary_indices.clear();

		// plane
		scan->planar_regions.clear();
		scan->plane_indices.clear();

		// rgb_comparator
		//pcl::RGBPlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr rgb_comparator;
		//rgb_comparator.reset (new pcl::RGBPlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal> ());
		//rgb_comparator->setColorThreshold(20);
		//pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setComparator(rgb_comparator);

		// plane segmentation;
		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputNormals(scan->normal_cloud);
		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputCloud(scan->point_cloud);
		pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::segmentAndRefine(scan->planar_regions, model_coefficients, scan->plane_indices, segment_label_cloud, segment_indices, segment_boundary_indices);

//		std::cout<<"planar regions:"<<std::endl;
//		for(size_t i=0;i<scan->planar_regions.size();i++)
//		{
//			Eigen::Vector4f co=scan->planar_regions[i].getCoefficients();
//			std::cout<<co.transpose()<<std::endl;
//		}
//		std::cout<<"similarity between"<<std::endl;
//		for(size_t i=0;i<scan->planar_regions.size();i++)
//		{
//			for(size_t j=0;j<scan->planar_regions.size();j++)
//			{
//				Eigen::Vector4f coi=scan->planar_regions[i].getCoefficients();
//				Eigen::Vector4f coj=scan->planar_regions[j].getCoefficients();
//				std::cout<<coi.block<3,1>(0,0).transpose()*coj.block<3,1>(0,0)<<","<<fabs(fabs(coi(3))-fabs(coj(3)))<<"\t";
//			}
//			std::cout<<std::endl;
//		}

		// generate the plane labels;
		scan->plane_label_cloud=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		pcl::Label invalid_pt;
		invalid_pt.label = unsigned (0);
		scan->plane_label_cloud->points.resize (scan->point_cloud->size(), invalid_pt);
		scan->plane_label_cloud->width = scan->point_cloud->width;
		scan->plane_label_cloud->height = scan->point_cloud->height;
		for(size_t i=0;i<scan->plane_indices.size();i++)
		{
			for(size_t j=0;j<scan->plane_indices[i].indices.size();j++)
			{
				scan->plane_label_cloud->at(scan->plane_indices[i].indices[j]).label=i+1;
			}
		}
	}


	void PlaneFeatureExtraction::extractEdges(Scan *scan)
	{
		// edge
		scan->edge_label_cloud=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		scan->edge_indices.clear();

		// for edge detection;
		// change the invalid depth in scan->point_cloud from zero to infinite;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		*cloud_tmp=*scan->point_cloud;
		for(size_t i=0;i<480;i++)
		{
			for(size_t j=0;j<640;j++)
			{
				double dep=cloud_tmp->points[640*i+j].z;
				if(std::abs(dep)<1e-4)
				{
					cloud_tmp->points[640*i+j].z=std::numeric_limits<double>::max();
				}
			}
		}

		// edge detection;
		if (getEdgeType () & EDGELABEL_HIGH_CURVATURE)
		{
			pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputNormals(scan->normal_cloud);
		}
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputCloud(cloud_tmp);
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::compute(*scan->edge_label_cloud, scan->edge_indices);

//		// fill scan->salient_points;
//		// xyz and type;
//		scan->salient_points.resize(scan->edge_indices.size());
//		for(size_t i=1;i<scan->edge_indices.size();i++)
//		{
//			scan->salient_points[i].resize(scan->edge_indices[i].indices.size());
//			for(size_t j=0;j<scan->edge_indices[i].indices.size();j++)
//			{
//				scan->salient_points[i][j]=new SalientPoint;
//				scan->salient_points[i][j]->xyz(0)=scan->point_cloud->at(scan->edge_indices[i].indices[j]).x;
//				scan->salient_points[i][j]->xyz(1)=scan->point_cloud->at(scan->edge_indices[i].indices[j]).y;
//				scan->salient_points[i][j]->xyz(2)=scan->point_cloud->at(scan->edge_indices[i].indices[j]).z;
//				if(i==1)
//					scan->salient_points[i][j]->type=SalientPoint::TYPE::OCCLUDING;
//				else if(i==2)
//					scan->salient_points[i][j]->type=SalientPoint::TYPE::OCCLUDED;
//				else if(i==3)
//					scan->salient_points[i][j]->type=SalientPoint::TYPE::HIGH_CURVATURE;
//			}
//		}
	}

	void PlaneFeatureExtraction::associateEdges(Scan *scan)
	{
//		ofstream fp;
//		fp.open("edge_points.txt",std::ios::out);

		ANNkd_tree *kdtree;
		ANNpointArray occluded_points;
		ANNpoint query_point=annAllocPt(2);
		ANNidxArray index;
		ANNdistArray distance;
		index=new ANNidx[1];
		distance=new ANNdist[1];
		// scan->edge_indices[1] - occluding points;
		// scan->edge_indices[2] - occluded points;
		occluded_points=annAllocPts(scan->edge_indices[2].indices.size(),2);
		for(size_t i=0;i<scan->edge_indices[2].indices.size();i++)
		{
			int idx=scan->edge_indices[2].indices[i];
			occluded_points[i][0]=scan->pixel_cloud->at(idx).x;
			occluded_points[i][1]=scan->pixel_cloud->at(idx).y;
		}
		// build the kd-tree using the occluded edge points;
		kdtree=new ANNkd_tree(occluded_points,scan->edge_indices[2].indices.size(),2);

		// use the occluding edge point as the query point,
		// search for the nearest neighbor in the occluded edge;
		scan->occluding_correspondences.indices.resize(scan->edge_indices[1].indices.size());
		scan->occluding_correspondences_distance.resize(scan->edge_indices[1].indices.size());
		scan->projective_rays.resize(scan->edge_indices[1].indices.size());

		scan->occluded_correspondences.indices.resize(scan->edge_indices[2].indices.size());
		for(size_t i=0;i<scan->occluded_correspondences.indices.size();i++)
		{
			scan->occluded_correspondences.indices[i]=-1;
		}

		for(size_t i=0;i<scan->edge_indices[1].indices.size();i++)
		{
			int idx=scan->edge_indices[1].indices[i];//occluding;
			query_point[0]=scan->pixel_cloud->at(idx).x;
			query_point[1]=scan->pixel_cloud->at(idx).y;
			kdtree->annkSearch(query_point,1,index,distance,0);
			int idx2=scan->edge_indices[2].indices[index[0]];//occluded;

//			scan->salient_points[1][i]->ptr_plane=scan->salient_points[2][index[0]];
//			scan->salient_points[2][index[0]]->ptr_plane=scan->salient_points[1][i];
//
//			scan->salient_points[1][i]->proj_ray(0)=scan->point_cloud->at(idx2).x-scan->point_cloud->at(idx).x;
//			scan->salient_points[1][i]->proj_ray(1)=scan->point_cloud->at(idx2).y-scan->point_cloud->at(idx).y;
//			scan->salient_points[1][i]->proj_ray(2)=scan->point_cloud->at(idx2).z-scan->point_cloud->at(idx).z;
//
//			scan->salient_points[2][index[0]]->proj_ray=-scan->salient_points[1][i]->proj_ray;

			if(distance[0]<5)
			{
				scan->occluding_correspondences.indices[i]=index[0];
				scan->occluding_correspondences_distance[i]=distance[0];
				scan->projective_rays[i](0)=scan->point_cloud->at(idx2).x-scan->point_cloud->at(idx).x;
				scan->projective_rays[i](1)=scan->point_cloud->at(idx2).y-scan->point_cloud->at(idx).y;
				scan->projective_rays[i](2)=scan->point_cloud->at(idx2).z-scan->point_cloud->at(idx).z;

				scan->occluded_correspondences.indices[index[0]]=i;
//				int idx2=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[i]];
//				fp<<scan->pixel_cloud->at(idx).x<<"\t"<<scan->pixel_cloud->at(idx).y<<"\t"<<scan->pixel_cloud->at(idx2).x<<"\t"<<scan->pixel_cloud->at(idx2).y<<"\t";
//				fp<<distance[0]<<"\t"<<scan->occluding_correspondences.indices[i]<<std::endl;
//				scan->projective_rays[i](0)=scan->point_cloud->at().x-scan->point_cloud->at().x;
			}
			else
			{
				scan->occluding_correspondences.indices[i]=-1;
			}
		}

//////////////////////////////////////////////////////////////////////
//			cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
//			for (size_t i=0;i<scan->edge_label_cloud->height;i++)
//			{
//				for (size_t j=0;j<scan->edge_label_cloud->width;j++)
//				{
////					if(scan->edge_label_cloud->at(j,i).label==1)
////					{
////						plane_img.at<cv::Vec3b>(i,j)[0]=255;
////						plane_img.at<cv::Vec3b>(i,j)[1]=0;
////						plane_img.at<cv::Vec3b>(i,j)[2]=0;
////					}
//					if(scan->edge_label_cloud->at(j,i).label==2)
//					{
//						plane_img.at<cv::Vec3b>(i,j)[0]=0;
//						plane_img.at<cv::Vec3b>(i,j)[1]=0;
//						plane_img.at<cv::Vec3b>(i,j)[2]=255;
//					}
//					if(scan->edge_label_cloud->at(j,i).label==4)
//					{
//						plane_img.at<cv::Vec3b>(i,j)[0]=0;
//						plane_img.at<cv::Vec3b>(i,j)[1]=255;
//						plane_img.at<cv::Vec3b>(i,j)[2]=0;
//					}
//				}
//			}
//			cv::imshow("img",plane_img);
//			cv::waitKey(0);
//			for(size_t i=0;i<scan->occluding_correspondences.indices.size();i++)
//			{
//				cv::Point pt1,pt2;
////				scan->edge_indices[1].indices[i]
//				int idx1=scan->edge_indices[1].indices[i];
//				int idx2=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[i]];
//				if(scan->occluding_correspondences.indices[i]==-1)
//					continue;
//				pt1.x=scan->pixel_cloud->at(idx1).x;
//				pt1.y=scan->pixel_cloud->at(idx1).y;
//				pt2.x=scan->pixel_cloud->at(idx2).x;
//				pt2.y=scan->pixel_cloud->at(idx2).y;
//				line(plane_img,pt1,pt2,CV_RGB(255,255,255));
////				fp<<pt1.x<<" "<<pt1.y<<" "<<pt2.x<<" "<<pt2.y<<std::endl;
//			}
//			cv::imshow("img",plane_img);
//			cv::waitKey(0);
/////////////////////////////////////////////////////////////////////////////////
//
//
//
//		fp.close();
//				sprintf(id,"%d%d",iterations,i);
//				vis->removeShape(id);
//				vis->addLine<pcl::PointXYZRGBA>(tmp_point,scan->point_cloud->at(index_cur),0,1,0,id);
	}

	void PlaneFeatureExtraction::associateEdgePlane(Scan *scan)
	{
//		std::vector<pcl::PointIndices> edge_point_indices=scan->edge_indices;
//		pcl::PointIndices scan->occluded_corresponding_plane;
//		scan->occluded_corresponding_plane.indices.clear();
//		scan->occluded_corresponding_plane.indices=scan->edge_indices[2].indices;
//		for(size_t i=0;i<scan->occluded_corresponding_plane.indices.size();i++)
//		{
//			scan->occluded_corresponding_plane.indices[i]=scan->plane_label_cloud->at(scan->occluded_corresponding_plane.indices[i]).label;
//		}
//		return;


//		ofstream fp;
//		fp.open("assiciatePlane.txt",std::ios::out);

		scan->edge_corresponding_plane.resize(scan->edge_indices.size());
		for(size_t i=0;i<scan->edge_indices.size();i++)
		{
			scan->edge_corresponding_plane[i].indices.resize(scan->edge_indices[i].indices.size());
			for(size_t j=0;j<scan->edge_indices[i].indices.size();j++)
			{
				scan->edge_corresponding_plane[i].indices[j]=scan->plane_label_cloud->at(scan->edge_indices[i].indices[j]).label-1;
				if(scan->edge_corresponding_plane[i].indices[j]>=0)
					continue;
				int u=scan->pixel_cloud->at(scan->edge_indices[i].indices[j]).x;
				int v=scan->pixel_cloud->at(scan->edge_indices[i].indices[j]).y;
				double min=99999999, dist_norm;
				int idx_pln=-1;
				Eigen::Vector3d dist;
				for(int m=u-3;m<=u+3;m++)
				{
					for(int n=v-3;n<=v+3;n++)
					{
						if(m<0 || n<0 || m>=480 || n>=640)
							continue;
						int idx=m*640+n;
						if(scan->plane_label_cloud->at(idx).label>0)
						{
							dist(0)=scan->point_cloud->at(idx).x-scan->point_cloud->at(scan->edge_indices[i].indices[j]).x;
							dist(1)=scan->point_cloud->at(idx).y-scan->point_cloud->at(scan->edge_indices[i].indices[j]).y;
							dist(2)=scan->point_cloud->at(idx).z-scan->point_cloud->at(scan->edge_indices[i].indices[j]).z;
							dist_norm=dist.norm();
							if(dist_norm<min)
							{
								min=dist_norm;
								if(dist_norm<0.01)
									idx_pln=scan->plane_label_cloud->at(idx).label-1;
							}
						}
					}
				}
				scan->edge_corresponding_plane[i].indices[j]=idx_pln;
			}
		}
//		fp.close();

//		Eigen::Vector4f plane_params, point;
//		point(3)=1;
//
//		for(size_t i=0;i<scan->edge_indices[2].indices.size();i++)
//		{
//			for(size_t j=0;j<scan->planar_regions.size();j++)
//			{
//				plane_params=scan->planar_regions[j].getCoefficients();
//				int idx=scan->edge_indices[2].indices[i];
//
//				point(0)=scan->point_cloud->at(idx).x;
//				point(1)=scan->point_cloud->at(idx).y;
//				point(2)=scan->point_cloud->at(idx).z;
//
//				double dist=point.transpose()*plane_params;
//			}
//		}
	}

	void PlaneFeatureExtraction::generateObservation(Scan *scan)
	{
		timeval start, end;
		double timeused;

		scan->Tcg=new Transform;

//		ofstream fp;
//		fp.open("extractPlanes.txt",std::ios::out);
//
		if(debug)
			std::cout<<"segmenting planes..."<<std::endl;

		gettimeofday(&start,NULL);
		segmentPlanes(scan);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time segmenting planes:"<<timeused<<std::endl;

		if(debug)
			std::cout<<"extracting edges..."<<std::endl;

		gettimeofday(&start,NULL);
		extractEdges(scan);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time extracting edges:"<<timeused<<std::endl;

		if(debug)
			std::cout<<"associating edge points..."<<std::endl;

		gettimeofday(&start,NULL);
		associateEdges(scan);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time assiciating edges:"<<timeused<<std::endl;

		if(debug)
			std::cout<<"associating edge points to  planes..."<<std::endl;

		gettimeofday(&start,NULL);
		associateEdgePlane(scan);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time associating edges to planes:"<<timeused<<std::endl;

		if(debug)
			std::cout<<"done!"<<std::endl;
	}


	void PlaneFeatureExtraction::generatePlaneFeature(Scan *scan)
	{
		timeval start, end;
		double timeused;

		// now all the neccessary information is in scan;
		scan->observed_planes.resize(scan->planar_regions.size());

		if(debug)
			std::cout<<"resize "<<scan->observed_planes.size()<<" observed planes."<<std::endl;

//		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		gettimeofday(&start,NULL);
		for(size_t i=0;i<scan->planar_regions.size();i++)
		{
			if(debug)
				std::cout<<std::endl<<"generating the "<<i<<"th plane feature."<<std::endl;

			scan->observed_planes[i]=new Plane;

			scan->observed_planes[i]->index=i;
			scan->observed_planes[i]->points=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >();
			scan->observed_planes[i]->points->resize(scan->plane_indices[i].indices.size());
			for(size_t j=0;j<scan->plane_indices[i].indices.size();j++)
			{
				scan->observed_planes[i]->points->at(j).x=scan->point_cloud->at(scan->plane_indices[i].indices[j]).x;
				scan->observed_planes[i]->points->at(j).y=scan->point_cloud->at(scan->plane_indices[i].indices[j]).y;
				scan->observed_planes[i]->points->at(j).z=scan->point_cloud->at(scan->plane_indices[i].indices[j]).z;
			}

			if(debug)
				std::cout<<"points "<<scan->observed_planes[i]->points->size()<<std::endl;

			// coefficients;
			Eigen::Vector4f vec4f=scan->planar_regions[i].getCoefficients();
			if(vec4f(3)<0)
			{
				for(size_t j=0;j<4;j++)
				{
					scan->observed_planes[i]->coefficients(j)=-vec4f(j);
				}
			}
			else
			{
				for(size_t j=0;j<4;j++)
				{
					scan->observed_planes[i]->coefficients(j)=vec4f(j);
				}
			}
			scan->observed_planes[i]->normal=scan->observed_planes[i]->coefficients.block<3,1>(0,0);

			if(debug)
				std::cout<<"coefficients: "<<scan->observed_planes[i]->coefficients.transpose()<<std::endl;

			// centroid_xyz;
			Eigen::Vector3f vec3f=scan->planar_regions[i].getCentroid();
			for(size_t j=0;j<3;j++)
			{
				scan->observed_planes[i]->centroid_xyz(j)=vec3f(j);
			}

			if(debug)
				std::cout<<"centroid_xyz: "<<scan->observed_planes[i]->centroid_xyz.transpose()<<std::endl;

			// covariance_xyz;
			Eigen::Matrix3f mat3f=scan->planar_regions[i].getCovariance();
			for(size_t j=0;j<3;j++)
			{
				for(size_t k=0;k<3;k++)
				{
					scan->observed_planes[i]->covariance_xyz(j,k)=mat3f(j,k);
				}
			}

			if(debug)
				std::cout<<"covariance_xyz: "<<std::endl<<scan->observed_planes[i]->covariance_xyz<<std::endl;

			// curvature;
			EIGEN_ALIGN16 Eigen::Vector3d eigen_value;
			EIGEN_ALIGN16 Eigen::Matrix3d eigen_vector;
			pcl::eigen33 (scan->observed_planes[i]->covariance_xyz, eigen_vector, eigen_value);
			double eig_sum = scan->observed_planes[i]->covariance_xyz(0,0)+scan->observed_planes[i]->covariance_xyz(1,1)
							+scan->observed_planes[i]->covariance_xyz(2,2);
//			if(eigen_value(0)<0)
			{
//				eigen_value(0)=fabs(eigen_value(0));
				Eigen::Matrix3d eigen_values;
				eigen_values.setZero();
				for(size_t j=0;j<3;j++)
				{
					eigen_values(j,j)=fabs(eigen_value(j));
				}
				scan->observed_planes[i]->covariance_xyz=eigen_vector*eigen_values*eigen_vector.transpose();
			}
			scan->observed_planes[i]->eigenvalues_xyz=eigen_value;
			scan->observed_planes[i]->eigenvectors_xyz=eigen_vector;
			if(debug)
				std::cout<<"transformed covariance_xyz: "<<std::endl<<scan->observed_planes[i]->covariance_xyz<<std::endl;

			if (eig_sum != 0)
				scan->observed_planes[i]->curvature= fabsf (eigen_value(0) / eig_sum);
			else
				scan->observed_planes[i]->curvature = -1;

			if(debug)
				std::cout<<"curvature: "<<scan->observed_planes[i]->curvature<<std::endl;

			// count;
			scan->observed_planes[i]->count=scan->planar_regions[i].getCount();

			// size;
			scan->observed_planes[i]->size=sqrt(eigen_value(2));

			if(debug)
				std::cout<<"count: "<<scan->observed_planes[i]->count<<std::endl;

//			// contour;
//			pcl::PointCloud<pcl::PointXYZRGBA> contour;
//			contour.points=scan->planar_regions[i].getContour();
//			scan->observed_planes[i]->contour.resize(contour.size());
//			for(size_t j=0;j<contour.size();j++)
//			{
//				scan->observed_planes[i]->contour[j](0)=contour.at(j).x;
//				scan->observed_planes[i]->contour[j](1)=contour.at(j).y;
//				scan->observed_planes[i]->contour[j](2)=contour.at(j).z;
//			}
//
//			if(debug)
//				std::cout<<"contour size: "<<scan->observed_planes[i]->contour.size()<<std::endl;

			// centroid_rgb;
			// covariance_rgb;
			pcl::PointCloud<pcl::PointXYZ> cloud_rgb;
			pcl::PointXYZ rgb_tmp;
			Eigen::Vector4d vec4d;
			Eigen::Matrix3d mat3d=Eigen::Matrix3d::Zero();
			for(size_t j=0;j<scan->plane_indices[i].indices.size();j++)
			{
				rgb_tmp.x=double(scan->point_cloud->at(scan->plane_indices[i].indices[j]).r)/255;
				rgb_tmp.y=double(scan->point_cloud->at(scan->plane_indices[i].indices[j]).g)/255;
				rgb_tmp.z=double(scan->point_cloud->at(scan->plane_indices[i].indices[j]).b)/255;
				cloud_rgb.push_back(rgb_tmp);
			}
			pcl::computeMeanAndCovarianceMatrix(cloud_rgb,mat3d,vec4d);
			scan->observed_planes[i]->centroid_rgb=vec4d.block<3,1>(0,0);
			scan->observed_planes[i]->covariance_rgb=mat3d;

			if(debug)
			{
				std::cout<<"centroid_rgb: "<<scan->observed_planes[i]->centroid_rgb.transpose()<<std::endl;
				std::cout<<"covariance_rgb: "<<std::endl<<scan->observed_planes[i]->covariance_rgb<<std::endl;
			}
		}
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time generating plane features:"<<timeused<<std::endl;


		// salient_points, projective_rays;
		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			scan->observed_planes[i]->occluded_points_seq.resize(1);
			scan->observed_planes[i]->occluding_points_seq.resize(1);
			scan->observed_planes[i]->ptr_camera_pose_seq.resize(1);
			scan->observed_planes[i]->ptr_camera_pose_seq[0]=scan->Tcg_gt;
		}
		gettimeofday(&start,NULL);
		for(size_t i=1;i<scan->edge_indices.size();i++)
		{ // occluding, occluded, high curvature;
			for(size_t j=0;j<scan->edge_indices[i].indices.size();j++)
			{
				int idx_pln=scan->edge_corresponding_plane[i].indices[j];
				int idx_pt=scan->edge_indices[i].indices[j];

//				std::cout<<"idx:"<<idx_pln<<","<<idx_pt<<std::endl;

				if(idx_pln==-1)
					continue;
				if(scan->occluded_correspondences.indices[j]==-1)
					continue;
				if(scan->occluding_correspondences.indices[j]==-1)
					continue;
				
				SalientPoint *spt=new SalientPoint;
				spt->xyz(0)=scan->point_cloud->at(idx_pt).x;
				spt->xyz(1)=scan->point_cloud->at(idx_pt).y;
				spt->xyz(2)=scan->point_cloud->at(idx_pt).z;
				spt->plane_on=scan->observed_planes[idx_pln];

				Eigen::Vector3d vec3d;
				if(i==1) // occluding;
				{
					vec3d=spt->xyz;
//					scan->observed_planes[idx_pln]->real_contour.push_back(vec3d);
					vec3d=scan->projective_rays[j];
					spt->proj_ray=scan->projective_rays[j];
					spt->type=SalientPoint::TYPE::OCCLUDING;

					// if occluding point projects to another plane,
					// then it will be pushed into occluding_points_seq[0];
					if(scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[j]]>=0)
					{
						int idx_pt_occluded=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[j]];
						int idx_pln_occluded=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[j]];
						
//						std::cout<<idx_pt_occluded<<","<<idx_pln_occluded<<std::endl;

						SalientPoint *spt_occluded=new SalientPoint;
						spt_occluded->xyz(0)=scan->point_cloud->at(idx_pt_occluded).x;
						spt_occluded->xyz(1)=scan->point_cloud->at(idx_pt_occluded).y;
						spt_occluded->xyz(2)=scan->point_cloud->at(idx_pt_occluded).z;
						spt_occluded->proj_ray=-spt->proj_ray;
						spt_occluded->type=SalientPoint::TYPE::OCCLUDED;
						spt_occluded->plane_on=scan->observed_planes[idx_pln_occluded];
						spt_occluded->ptr_plane=spt;
						spt->ptr_plane=spt_occluded;
						scan->observed_planes[idx_pln_occluded]->occluded_points_seq[0].push_back(spt_occluded);
					}
					else
					{
						spt->ptr_plane=0;
					}
					scan->observed_planes[idx_pln]->occluding_points_seq[0].push_back(spt);
				}
				else if(i==2) // occluded;
				{
					vec3d=-scan->projective_rays[scan->occluded_correspondences.indices[j]];
					spt->proj_ray=-scan->projective_rays[scan->occluded_correspondences.indices[j]];
					spt->type=SalientPoint::TYPE::OCCLUDED;
					spt->ptr_plane=0;
					scan->observed_planes[idx_pln]->occluded_points_seq[0].push_back(spt);
				}
				else // high curvature;
				{//not used;
//					vec3d.setZero();
//					spt->proj_ray.setZero();
//					spt->type=SalientPoint::TYPE::HIGH_CURVATURE;
//					spt->ptr_plane=0;
//					scan->observed_planes[idx_pln]->high_curvature_points.push_back(spt);
				}
			}
		}
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		if(debug)
		std::cout<<"time extracting salient points:"<<timeused<<std::endl;
		
		if(debug)
		{
			std::cout<<std::endl<<"salient points:"<<std::endl;
			for(size_t i=0;i<scan->observed_planes.size();i++)
			{
//				std::cout<<"occluding_points: "<<scan->observed_planes[i]->occluding_points.size()<<std::endl;
//				std::cout<<"occluding_points_plane: "<<scan->observed_planes[i]->occluding_points_plane.size()<<std::endl;
				std::cout<<"occluding_points_seq[0]: "<<scan->observed_planes[i]->occluding_points_seq[0].size()<<std::endl;
				std::cout<<"occluded_points_seq[0]: "<<scan->observed_planes[i]->occluded_points_seq[0].size()<<std::endl;
			}
		}

//		fp.close();
	}
}

