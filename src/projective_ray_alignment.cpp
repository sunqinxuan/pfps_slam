/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-05-20 10:33
#
# Filename:		projective_ray_alignment.cpp
#
# Description: 
#
===============================================*/


#include "projective_ray_alignment.h"

namespace ulysses
{

	void EdgeSE3pra::computeError()
	{
		const g2o::VertexSE3Expmap* v  =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
		Eigen::Vector3d p_local = v->estimate().map ( p_global );
		_error=(p_local-_measurement)/p_local(2);
//		Eigen::Vector3d u=int_param.project(p_local);
//		int x=(int)u(0);
//		int y=(int)u(1);

//		ANNpoint query_point=annAllocPt(2);
//		ANNidxArray point_index;
//		ANNdistArray distance;
//		point_index=new ANNidx[1];
//		distance=new ANNdist[1];
//
//		query_point[0]=u(0);
//		query_point[1]=u(1);
//		kdtree->annkSearch(query_point,1,point_index,distance,0);
////		scan->pixel_cloud->at(scan->edge_indices[1].indices[point_index[0]]);
//
//		delete point_index;
//		delete distance;

//		// check if u is in the image domain;
//        if ( y-4<0 || ( y+4 ) >depth_image->cols || ( x-4 ) <0 || ( x+4 ) >depth_image->rows )
//		{
//			// for invalid pixel coordinate, set the _error to zero;
//			// is it reasonable??
//			_error ( 0,0 ) = 0.0;
//			this->setLevel ( 1 );
//		}
//		else
//		{
//			_error ( 0,0 ) = getPixelDepth(x,y) - _measurement;
//		}
		
//		if(debug)
//		{
//			cout<<"computeError p_local="<<p_local.transpose()<<endl;
////			cout<<"computeError p_cur="<<p_cur.transpose()<<endl;
//			cout<<"computeError error="<<_error.transpose()<<endl;
//		}
	}

/*
	void EdgeSE3pra::linearizeOplus()
	{
		if ( level() == 1 )
		{
			_jacobianOplusXi = Eigen::Matrix<double, 3, 6>::Zero();
			return;
		}
		g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
		Eigen::Vector3d p_local = vtx->estimate().map ( p_global );

//		double x = p_local[0];
//		double y = p_local[1];
//		double invz = 1.0/p_local[2];
//		double invz_2 = invz*invz;
//
//		float u = x*int_param.fx*invz + int_param.cx;
//		float v = y*int_param.fy*invz + int_param.cy;
//
//		// jacobian of pixel coordinates w.r.t. \xi (se3);
//		// \frac {\partial u} {\partial \xi};
//		// dimension 2*6;
//		Eigen::Matrix<double, 2, 6> J_u_xi;
//
//		J_u_xi ( 0,0 ) = - x*y*invz_2 *int_param.fx;
//		J_u_xi ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *int_param.fx;
//		J_u_xi ( 0,2 ) = - y*invz *int_param.fx;
//		J_u_xi ( 0,3 ) = invz *int_param.fx;
//		J_u_xi ( 0,4 ) = 0;
//		J_u_xi ( 0,5 ) = -x*invz_2 *int_param.fx;
//
//		J_u_xi ( 1,0 ) = - ( 1+y*y*invz_2 ) *int_param.fy;
//		J_u_xi ( 1,1 ) = x*y*invz_2 *int_param.fy;
//		J_u_xi ( 1,2 ) = x*invz *int_param.fy;
//		J_u_xi ( 1,3 ) = 0;
//		J_u_xi ( 1,4 ) = invz *int_param.fy;
//		J_u_xi ( 1,5 ) = -y*invz_2 *int_param.fy;
//
//		// jacobian of depth value w.r.t. pixel coordinates;
//		// i.e., the gradient at the pixel coordinates;
//		Eigen::Matrix<double, 1, 2> J_depth_u;
//
//		J_depth_u ( 0,0 ) = ( getPixelDepth ( u+1,v )-getPixelDepth ( u-1,v ) ) /2;
//		J_depth_u ( 0,1 ) = ( getPixelDepth ( u,v+1 )-getPixelDepth ( u,v-1 ) ) /2;
//
//		_jacobianOplusXi = J_depth_u*J_u_xi;
//
//		if(debug)
//		{
//			cout<<"linearizeOplus p_local="<<p_local.transpose()<<endl;
//			cout<<"linearizeOplus pixel="<<u<<" "<<v<<endl;
//			cout<<"linearizeOplus J_u_xi="<<endl<<J_u_xi<<endl;
//			cout<<"linearizeOplus J_depth_u="<<endl<<J_depth_u<<endl;
//			cout<<"linearizeOplus _jacobianOplusXi="<<endl<<_jacobianOplusXi<<endl;
//		}

		double x = p_local[0];
		double y = p_local[1];
		double z = p_local[2];

		Eigen::Matrix<double, 3, 6> J_p_xi;

		J_p_xi (0,0) = 1;
		J_p_xi (0,1) = 0;
		J_p_xi (0,2) = 0;
		J_p_xi (0,3) = 0;
		J_p_xi (0,4) = -z;
		J_p_xi (0,5) = y;

		J_p_xi (1,0) = 0;
		J_p_xi (1,1) = 1;
		J_p_xi (1,2) = 0;
		J_p_xi (1,3) = z;
		J_p_xi (1,4) = 0;
		J_p_xi (1,5) = -x;

		J_p_xi (2,0) = 0;
		J_p_xi (2,1) = 0;
		J_p_xi (2,2) = 1;
		J_p_xi (2,3) = -y;
		J_p_xi (2,4) = x;
		J_p_xi (2,5) = 0;

		_jacobianOplusXi = J_p_xi;

		if(debug)
		{
			cout<<"linearizeOplus p_local="<<p_local.transpose()<<endl;
			cout<<"linearizeOplus _jacobianOplusXi="<<endl<<_jacobianOplusXi<<endl;
		}
	}
*/

//	double EdgeSE3pra::getPixelDepth(int u, int v)
//	{
//		unsigned char *depth_data_ptr=depth_image->data+depth_image->step[0]*u+depth_image->step[1]*v;
//		unsigned short *depth_ptr;
//		memcpy(depth_ptr,depth_data_ptr,2);
//		return (double)*depth_ptr/5000;
//	}

	void ProjectiveRayAlignment::extractEdgePoints(Scan *scan)
	{
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
	}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "q" && event.keyDown ())
  {
	  viewer->close();
  }
}


	void ProjectiveRayAlignment::align(Scan *scan_ref, Scan *scan_cur, Eigen::Isometry3d& Tcr, IntrinsicParam int_param,
										boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
//		std::ofstream fp;
//		fp.open("align.txt", std::ios::app);

//		boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//		vis->setBackgroundColor (0, 0, 0);
//		vis->addCoordinateSystem (1.0);
//		vis->initCameraParameters ();
//		vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get ());

//		pcl::PointIndices nn_indices;
		std::vector<int> indices;
		std::vector<double> dist;
		char id[20];

		// g2o initialization;
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
		// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose( true );
//		std::cout<<"g2o initialized."<<std::endl;


//		// build the kdtree structure for the occluding edge points in current scan;
//		// using the pixel coordinates of occluding edge points in image domain;
//		ANNkd_tree *kdtree;
//		ANNpointArray cur_occluding;
//		ANNpoint query_point=annAllocPt(2);
//		ANNidxArray point_index;
//		ANNdistArray distance;
//		point_index=new ANNidx[1];
//		distance=new ANNdist[1];
//		cur_occluding=annAllocPts(scan_cur->edge_indices[1].indices.size(),2);
//		for(int i=0;i<scan_cur->edge_indices[1].indices.size();i++)
//		{
//			int index=scan_cur->edge_indices[1].indices[i];
//			cur_occluding[i][0]=scan_cur->pixel_cloud->at(index).x;
//			cur_occluding[i][1]=scan_cur->pixel_cloud->at(index).y;
//		}
//		kdtree=new ANNkd_tree(cur_occluding,scan_cur->edge_indices[1].indices.size(),2);

		// build the kdtree structure for the occluding edge points in current scan;
		// using the 3d coordinates of occluding edge points in camera frame;
		ANNkd_tree *kdtree;
		ANNpointArray cur_occluding;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray point_index;
		ANNdistArray distance;
		point_index=new ANNidx[1];
		distance=new ANNdist[1];
		cur_occluding=annAllocPts(scan_cur->edge_indices[1].indices.size(),3);
		for(int i=0;i<scan_cur->edge_indices[1].indices.size();i++)
		{
			int index=scan_cur->edge_indices[1].indices[i];
			cur_occluding[i][0]=scan_cur->point_cloud->at(index).x;
			cur_occluding[i][1]=scan_cur->point_cloud->at(index).y;
			cur_occluding[i][2]=scan_cur->point_cloud->at(index).z;
		}
		kdtree=new ANNkd_tree(cur_occluding,scan_cur->edge_indices[1].indices.size(),3);

		int iterations = 0;
		bool converged = false;
		double chi2_pre=99999999999, chi2=99999999999;
		Eigen::Isometry3d Tcr_tmp = Eigen::Isometry3d::Identity();

		while(iterations<10 && !converged)
		{
			iterations++;
//			std::cout<<"**************************************************"<<std::endl;
//			std::cout<<"this is the "<<iterations<<"th iteration of icp..."<<std::endl;

//			fp<<"**************************************************"<<std::endl;
//			fp<<"this is the "<<iterations<<"th iteration of icp..."<<std::endl;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.rotation(), Tcr.translation() ) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );
//			std::cout<<"vertex added; initial guess of Tcr is:"<<std::endl<<Tcr.matrix()<<std::endl;

			Eigen::Vector3d vec_3d_ref, vec_3d_cur;
			Eigen::Vector3d vec_3d_pixel;
//			nn_indices.indices.clear();
			indices.clear();
			for(size_t i=0;i<scan_ref->edge_indices[1].indices.size();i++)
			{
	//			// index of current point (query point) in reference scan;
	//			int index=scan_ref->edge_indices[1].indices[i];
	//			// 3d coordinate in ref frame;
	//			vec_3d_ref(0)=scan_ref->point_cloud->at(index).x;
	//			vec_3d_ref(1)=scan_ref->point_cloud->at(index).y;
	//			vec_3d_ref(2)=scan_ref->point_cloud->at(index).z;
	//			// project the 3d point of ref scan into the cur scan;
	//			vec_3d_ref=Tcr.rotation()*vec_3d_ref+Tcr.translation();
	//			// project the 3d point in cur scan onto the image domain;
	//			// yielding the homogeneous pixel coordinates;
	//			vec_3d_pixel=int_param.project(vec_3d_ref);
	//			query_point[0]=vec_3d_pixel(0);//scan_ref->pixel_cloud->at(index).x;//
	//			query_point[1]=vec_3d_pixel(1);//scan_ref->pixel_cloud->at(index).y;//
	//			kdtree->annkSearch(query_point,1,point_index,distance,0);

				// index of current point (query point) in reference scan;
				int index=scan_ref->edge_indices[1].indices[i];
				// 3d coordinate in ref frame;
				vec_3d_ref(0)=scan_ref->point_cloud->at(index).x;
				vec_3d_ref(1)=scan_ref->point_cloud->at(index).y;
				vec_3d_ref(2)=scan_ref->point_cloud->at(index).z;
				// project the 3d point of ref scan into the cur scan;
				vec_3d_ref=Tcr.rotation()*vec_3d_ref+Tcr.translation();
				// project the 3d point in cur scan onto the image domain;
				// yielding the homogeneous pixel coordinates;
	//			vec_3d_pixel=int_param.project(vec_3d_ref);
				query_point[0]=vec_3d_ref(0);//scan_ref->pixel_cloud->at(index).x;//
				query_point[1]=vec_3d_ref(1);//scan_ref->pixel_cloud->at(index).y;//
				query_point[2]=vec_3d_ref(2);//scan_ref->pixel_cloud->at(index).y;//
				kdtree->annkSearch(query_point,1,point_index,distance,0);
//				nn_indices.indices.push_back(point_index[0]);
				indices.push_back(point_index[0]);
				dist.push_back(distance[0]);

//				if(distance[0]>0.1)
//				{
//					fp<<" "<<distance[0];
//					continue;
//				}

//				vec_3d_ref(0)=scan_ref->point_cloud->at(index).x;
//				vec_3d_ref(1)=scan_ref->point_cloud->at(index).y;
//				vec_3d_ref(2)=scan_ref->point_cloud->at(index).z;

				index=scan_cur->edge_indices[1].indices[point_index[0]];
				vec_3d_cur(0)=scan_cur->point_cloud->at(index).x;
				vec_3d_cur(1)=scan_cur->point_cloud->at(index).y;
				vec_3d_cur(2)=scan_cur->point_cloud->at(index).z;

				EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//, vec_3d_cur); // scan_cur, kdtree, &(scan_ref->img_depth));
				edge->setVertex(0,pose);
				edge->setMeasurement(vec_3d_cur);
				edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
				edge->setId(i);
//				edge->setDebug(true);
				optimizer.addEdge(edge);
			}
			
//			fp<<std::endl<<std::endl;

			optimizer.initializeOptimization();
			int result=optimizer.optimize ( 30 );
			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
//				std::cout<<"chi2_pre="<<chi2_pre<<std::endl;
//				std::cout<<"chi2="<<chi2<<std::endl;
				break;
			}

			Tcr_tmp = pose->estimate();
			Tcr=Tcr_tmp*Tcr;
//			std::cout<<"Tcr_tmp="<<std::endl<<Tcr_tmp.matrix() <<std::endl;
//			std::cout<<"Tcr="<<std::endl<<Tcr.matrix() <<std::endl;
//			if(abs(chi2_pre-chi2)/chi2<0.1)
//				converged=true;



			// *****************************************************************************************
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointXYZRGBA tmp_point;
			vis->removeAllPointClouds();
			Eigen::Matrix4f trans;
			for(size_t i=0;i<4;i++)
			{
				for(size_t j=0;j<4;j++)
				{
					trans(i,j)=Tcr.matrix()(i,j);
				}
			}
/*
			// show the edge points before transforming;
			cloud_show->clear();
			for(size_t i=0;i<scan_ref->edge_indices[1].indices.size();i++)
			{
				int index=scan_ref->edge_indices[1].indices[i];
				int index_cur=scan_cur->edge_indices[1].indices[indices[i]];
				tmp_point=scan_ref->point_cloud->at(index);
				tmp_point.r=255;
				tmp_point.g=0;
				tmp_point.b=0;
				cloud_show->push_back(tmp_point);

				if(dist[i]>0.1)
				{
					continue;
				}

				sprintf(id,"%d%d",iterations,i);
				vis->removeShape(id);
//				if(dist[i]>0.01)
				vis->addLine<pcl::PointXYZRGBA>(tmp_point,scan_cur->point_cloud->at(index_cur),0,1,0,id);
			}
			for(size_t i=0;i<scan_cur->edge_indices[1].indices.size();i++)
			{
				int index=scan_cur->edge_indices[1].indices[i];
				tmp_point=scan_cur->point_cloud->at(index);
				tmp_point.r=0;
				tmp_point.g=0;
				tmp_point.b=255;
				cloud_show->push_back(tmp_point);
			}
			if (!vis->updatePointCloud (cloud_show, "point")) //<pcl::PointXYZRGBA>
				vis->addPointCloud (cloud_show,"point");// "sample cloud");
			vis->spin();


			// show the edge points after transforming;
			cloud_show->clear();
			vis->removeAllPointClouds();
//			transformPointCloud(*scan_ref->point_cloud,*scan_ref->point_cloud,trans);
			for(size_t i=0;i<scan_ref->edge_indices[1].indices.size();i++)
			{
				int index=scan_ref->edge_indices[1].indices[i];
				int index_cur=scan_cur->edge_indices[1].indices[indices[i]];
				tmp_point=scan_ref->point_cloud->at(index);
				tmp_point.r=255;
				tmp_point.g=0;
				tmp_point.b=0;
				cloud_show->push_back(tmp_point);
//
//				if(dist[i]>0.1)
//				{
//					continue;
//				}
//
//				sprintf(id,"%d%d",iterations,i);
//				vis->removeShape(id);
//				vis->addLine<pcl::PointXYZRGBA>(tmp_point,scan_cur->point_cloud->at(index_cur),0,1,0,id);
			}
			transformPointCloud(*cloud_show,*cloud_show,trans);
			for(size_t i=0;i<scan_cur->edge_indices[1].indices.size();i++)
			{
				int index=scan_cur->edge_indices[1].indices[i];
				tmp_point=scan_cur->point_cloud->at(index);
				tmp_point.r=0;
				tmp_point.g=0;
				tmp_point.b=255;
				cloud_show->push_back(tmp_point);
			}
			if (!vis->updatePointCloud (cloud_show, "point")) //<pcl::PointXYZRGBA>
				vis->addPointCloud (cloud_show,"point");// "sample cloud");
			vis->spin();
			// *********************************************************************
*/
		}
		delete point_index;
		delete distance;
		annDeallocPts(cur_occluding);
//		fp.close();
//		vis->close();
	}

}

