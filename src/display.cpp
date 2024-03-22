/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-07-19 16:43
#
# Filename: display.cpp
#
# Description: 
#
===============================================*/

#include "display.h"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "q" && event.keyDown ())
  {
	  viewer->close();
  }
}



	void displayMapPlanes(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		for(size_t i=0;i<map->planes.size();i++)
		{
			sprintf(id,"map%d",i);
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color (map->planes[i]->points, red[i%13], grn[i%13], blu[i%13]);
			if (!vis->updatePointCloud (map->planes[i]->points, color, id))
				vis->addPointCloud (map->planes[i]->points, color, id);
		}
	}

	void displayMapPlanePoints(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr spt (new pcl::PointCloud<pcl::PointXYZRGBA>);

		vis->removeAllPointClouds();
		for(size_t i=0;i<map->planes.size();i++)
		{
			pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
			vg.setInputCloud(map->planes[i]->points);
			vg.setLeafSize(0.02,0.02,0.02);
			vg.filter(*map->planes[i]->points);
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (map->planes[i]->points, red[i%12], grn[i%12], blu[i%12]);
			sprintf(id,"map%d",i);
			if (!vis->updatePointCloud (map->planes[i]->points, color1, id))
				vis->addPointCloud (map->planes[i]->points, color1, id);

			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (spt, red[13], grn[13], blu[13]);
			sprintf(id,"spt%d",i);
			spt->resize(0);
			for(size_t k=0;k<map->planes[i]->occluding_points_seq.size();k++)
			{
				size_t N=spt->size();
				spt->resize(N+map->planes[i]->occluding_points_seq[k].size());
				for(size_t j=0;j<map->planes[i]->occluding_points_seq[k].size();j++)
				{
					spt->at(N+j).x=map->planes[i]->occluding_points_seq[k][j]->xyz(0);
					spt->at(N+j).y=map->planes[i]->occluding_points_seq[k][j]->xyz(1);
					spt->at(N+j).z=map->planes[i]->occluding_points_seq[k][j]->xyz(2);
				}
			}
			if (!vis->updatePointCloud (spt, color2, id))
				vis->addPointCloud (spt, color2, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);

//			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color3 (spt, red[12], grn[12], blu[12]);
//			sprintf(id,"spted%d",i);
//			spt->resize(0);
//			for(size_t k=0;k<map->planes[i]->occluded_points_seq.size();k++)
//			{
//				size_t N=spt->size();
//				spt->resize(N+map->planes[i]->occluded_points_seq[k].size());
//				for(size_t j=0;j<map->planes[i]->occluded_points_seq[k].size();j++)
//				{
//					spt->at(N+j).x=map->planes[i]->occluded_points_seq[k][j]->xyz(0);
//					spt->at(N+j).y=map->planes[i]->occluded_points_seq[k][j]->xyz(1);
//					spt->at(N+j).z=map->planes[i]->occluded_points_seq[k][j]->xyz(2);
//				}
//			}
//			if (!vis->updatePointCloud (spt, color3, id))
//				vis->addPointCloud (spt, color3, id);
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
		}
	}

	void displayScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		for(size_t i=0;i<scan->plane_indices.size();i++)
		{
			plane->clear();
			for(size_t j=0;j<scan->plane_indices[i].indices.size();j++)
			{
				int idx=scan->plane_indices[i].indices[j];
				plane->push_back(scan->point_cloud->points[idx]);
			}
			sprintf(id,"plane%d",i);
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[i%12], grn[i%12], blu[i%12]);
			if (!vis->updatePointCloud (plane, color1, id))
				vis->addPointCloud (plane, color1, id);
		}

		plane->clear();
		for(size_t j=0;j<scan->edge_indices[1].indices.size();j++)
		{
			int idx=scan->edge_indices[1].indices[j];
			plane->push_back(scan->point_cloud->points[idx]);
			if(scan->edge_corresponding_plane[1].indices[j]==-1)
			{
				plane->at(plane->size()-1).r=130;
				plane->at(plane->size()-1).g=0;
				plane->at(plane->size()-1).b=130;
			}
			else
			{
				plane->at(plane->size()-1).r=255;
				plane->at(plane->size()-1).g=255;
				plane->at(plane->size()-1).b=255;
			}
		}
		sprintf(id,"occluding");
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_occluding (plane, red[13], grn[13], blu[13]);
		if (!vis->updatePointCloud (plane, id))
			vis->addPointCloud (plane, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);

		plane->clear();
		for(size_t j=0;j<scan->edge_indices[2].indices.size();j++)
		{
			int idx=scan->edge_indices[2].indices[j];
			plane->push_back(scan->point_cloud->points[idx]);
			if(scan->edge_corresponding_plane[2].indices[j]==-1)
			{
				plane->at(plane->size()-1).r=0;
				plane->at(plane->size()-1).g=130;
				plane->at(plane->size()-1).b=130;
			}
			else
			{
				plane->at(plane->size()-1).r=130;
				plane->at(plane->size()-1).g=130;
				plane->at(plane->size()-1).b=130;
			}
		}
		sprintf(id,"occluded");
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_occluded (plane, red[12], grn[12], blu[12]);
		if (!vis->updatePointCloud (plane, id))
			vis->addPointCloud (plane, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
	}

	void displayScanPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		std::cout<<"displayScanPlanes ---"<<std::endl;
		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			sprintf(id,"plane%d",i);
//			sprintf(id,"plane%d",count);
//			count++;
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (scan->observed_planes[i]->points, red[i%12], grn[i%12], blu[i%12]);
			if (!vis->updatePointCloud (scan->observed_planes[i]->points, color1, id))
				vis->addPointCloud (scan->observed_planes[i]->points, color1, id);
			
			std::cout<<"observed_plane "<<i<<std::endl;
//					 <<"\toccluding_points - "<<scan->observed_planes[i]->occluding_points_seq[0].size()<<std::endl;
//			plane->resize(scan->observed_planes[i]->occluding_points_seq[0].size());
//			for(size_t j=0;j<scan->observed_planes[i]->occluding_points_seq[0].size();j++)
//			{
//				plane->at(j).x=scan->observed_planes[i]->occluding_points_seq[0][j]->xyz(0);
//				plane->at(j).y=scan->observed_planes[i]->occluding_points_seq[0][j]->xyz(1);
//				plane->at(j).z=scan->observed_planes[i]->occluding_points_seq[0][j]->xyz(2);
////				plane->at(j).r=255;
////				plane->at(j).g=255;
////				plane->at(j).b=255;
//			}
//			sprintf(id,"occluding%d",i);
//			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_occluding (plane, red[13], grn[13], blu[13]);
//			if (!vis->updatePointCloud (plane, color_occluding, id))
//				vis->addPointCloud (plane, color_occluding, id);
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);

//			std::cout<<"\toccluded_points - "<<scan->observed_planes[i]->occluded_points_seq[0].size()<<std::endl;
//			plane->resize(scan->observed_planes[i]->occluded_points_seq[0].size());
//			for(size_t j=0;j<scan->observed_planes[i]->occluded_points_seq[0].size();j++)
//			{
//				plane->at(j).x=scan->observed_planes[i]->occluded_points_seq[0][j]->xyz(0);
//				plane->at(j).y=scan->observed_planes[i]->occluded_points_seq[0][j]->xyz(1);
//				plane->at(j).z=scan->observed_planes[i]->occluded_points_seq[0][j]->xyz(2);
////				plane->at(j).r=130;
////				plane->at(j).g=130;
////				plane->at(j).b=130;
//			}
//			sprintf(id,"occluded%d",i);
//			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_occluded (plane, red[13], grn[13], blu[13]);
//			if (!vis->updatePointCloud (plane, color_occluded, id))
//				vis->addPointCloud (plane, color_occluded, id);
//			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
		}

		std::cout<<"\toccluding_points - "<<scan->edge_indices[1].indices.size()<<std::endl;
		plane->resize(scan->edge_indices[1].indices.size());
		for(size_t i=0;i<scan->edge_indices[1].indices.size();i++)
		{
			int idx=scan->edge_indices[1].indices[i];
			plane->at(i)=scan->point_cloud->at(idx);
		}
		sprintf(id,"occluding");
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_occluded (plane, red[13], grn[13], blu[13]);
		if (!vis->updatePointCloud (plane, color_occluded, id))
			vis->addPointCloud (plane, color_occluded, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
	}

	void display(Scan *scan_cur, Scan *scan_ref, Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		Transform Tgc;
		Eigen::Matrix4f trans;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
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
		// real_contour from map;
		cloud_show_tmp->resize(0);
		for(size_t i=0;i<map->planes.size();i++)
		{
			size_t N=cloud_show_tmp->size();
			cloud_show_tmp->resize(N+map->planes[i]->real_contour.size());
			for(size_t j=0;j<map->planes[i]->real_contour.size();j++)
			{
				cloud_show_tmp->at(N+j).x=map->planes[i]->real_contour[j](0);
				cloud_show_tmp->at(N+j).y=map->planes[i]->real_contour[j](1);
				cloud_show_tmp->at(N+j).z=map->planes[i]->real_contour[j](2);
				cloud_show_tmp->at(N+j).r=0;
				cloud_show_tmp->at(N+j).g=255;
				cloud_show_tmp->at(N+j).b=0;
			}
		}
		if (!vis->updatePointCloud (cloud_show_tmp, "real_contour_map"))
			vis->addPointCloud (cloud_show_tmp,"real_contour_map");
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "real_contour_map");
		// real_contour from cur_scan;
		cloud_show_tmp->resize(0);
		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
		{
			size_t N=cloud_show_tmp->size();
			cloud_show_tmp->resize(N+scan_cur->observed_planes[i]->real_contour.size());
			for(size_t j=0;j<scan_cur->observed_planes[i]->real_contour.size();j++)
			{
				cloud_show_tmp->at(N+j).x=scan_cur->observed_planes[i]->real_contour[j](0);
				cloud_show_tmp->at(N+j).y=scan_cur->observed_planes[i]->real_contour[j](1);
				cloud_show_tmp->at(N+j).z=scan_cur->observed_planes[i]->real_contour[j](2);
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
		// real_contour after trans;
		cloud_show_tmp->resize(0);
		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
		{
			size_t N=cloud_show_tmp->size();
			cloud_show_tmp->resize(N+scan_cur->observed_planes[i]->real_contour.size());
			for(size_t j=0;j<scan_cur->observed_planes[i]->real_contour.size();j++)
			{
				cloud_show_tmp->at(N+j).x=scan_cur->observed_planes[i]->real_contour[j](0);
				cloud_show_tmp->at(N+j).y=scan_cur->observed_planes[i]->real_contour[j](1);
				cloud_show_tmp->at(N+j).z=scan_cur->observed_planes[i]->real_contour[j](2);
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
	}


void displayPlanes (std::vector<Plane*> &planes, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  
  pcl::PointCloud<PointT>::Ptr slt_pt (new pcl::PointCloud<PointT>);

  //viewer->removeAllPointClouds();
  //viewer->removeAllShapes();
  for(size_t i=0;i<planes.size();i++)
  {
    Eigen::Vector3d centroid = planes[i]->centroid_xyz;
    Eigen::Vector4d model = planes[i]->coefficients;
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
	viewer->removeShape(name);
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

	slt_pt->clear();
	pcl::PointXYZRGBA pt,ptt;
	for(size_t j=0;j<planes[i]->contour.size();j++)
	{
//		if(planes[i]->salient_point[j]->point_ptr==0)
//			continue;
		pt.x=planes[i]->contour[j](0);
		pt.y=planes[i]->contour[j](1);
		pt.z=planes[i]->contour[j](2);
		slt_pt->push_back(pt);
	}
//	for(size_t j=0;j<planes[i]->salient_points.size();j++)
//	{
//		slt_pt->push_back(planes[i]->salient_points.at(j));
//	}
	sprintf (name, "planein_%02d", int (i));
	pcl::visualization::PointCloudColorHandlerCustom <PointT> color1 (slt_pt, red[i%6], grn[i%6], blu[i%6]);
	if(!viewer->updatePointCloud(slt_pt, color1, name))
	  viewer->addPointCloud (slt_pt, color1, name);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

	pcl::ModelCoefficients coefficients;
	coefficients.values.resize(4);
	for(size_t j=0;j<4;j++)
	{
		coefficients.values[j]=planes[i]->coefficients(j);
	}
    sprintf (name, "pl_%02d", int (i));
	viewer->removeShape(name);
	viewer->addPlane(coefficients,centroid(0),centroid(1),centroid(2),name);

//	for(size_t j=0;j<planes[i]->salient_point_seq[0].size();j++)
//	{
//		if(planes[i]->salient_point_seq[0][j]->point_ptr==0)
//			continue;
//		pt.x=planes[i]->salient_point_seq[0][j]->xyz(0);
//		pt.y=planes[i]->salient_point_seq[0][j]->xyz(1);
//		pt.z=planes[i]->salient_point_seq[0][j]->xyz(2);
//		ptt.x=planes[i]->salient_point_seq[0][j]->point_ptr->xyz(0);
//		ptt.y=planes[i]->salient_point_seq[0][j]->point_ptr->xyz(1);
//		ptt.z=planes[i]->salient_point_seq[0][j]->point_ptr->xyz(2);
//		sprintf(name,"%d%d",i,j);
//		viewer->removeShape(name);
//		viewer->addLine<pcl::PointXYZRGBA>(pt,ptt,red[i%6], grn[i%6], blu[i%6],name);
//	}

//	for(size_t j=0;j<planes[i]->projective_rays.size();j++)
//	{
//		if(planes[i]->projective_rays[j](0)==0 && planes[i]->projective_rays[j](1)==0 && planes[i]->projective_rays[j](2)==0)
//			continue;
//		pcl::PointXYZRGBA pt;
//		pt.x=planes[i]->projective_rays[j](0)+planes[i]->salient_points.at(j).x;
//		pt.y=planes[i]->projective_rays[j](1)+planes[i]->salient_points.at(j).y;
//		pt.z=planes[i]->projective_rays[j](2)+planes[i]->salient_points.at(j).z;
//		sprintf(name,"%d%d",i,j);
//		viewer->removeShape(name);
//		viewer->addLine<pcl::PointXYZRGBA>(planes[i]->salient_points.at(j),pt,red[i%6], grn[i%6], blu[i%6],name);
//	}

  }
}

void
displayPointsOnPlane (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
					  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,  
//					  std::vector<pcl::PointIndices> inlier_indices, 
					  pcl::PointIndices occluded_edge, pcl::PointIndices occluded_corresponding_plane)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < regions.size (); i++)
  {
    //Eigen::Vector3f centroid = regions[i].getCentroid ();
    //Eigen::Vector4f model = regions[i].getCoefficients ();
    //pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    //pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
    //                                   centroid[1] + (0.5f * model[1]),
    //                                   centroid[2] + (0.5f * model[2]));
    //sprintf (name, "normal_%d", unsigned (i));
    //viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
    
    //contour->points = regions[i].getContour ();
    //sprintf (name, "plane_%02d", int (i));
    //pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
    //if(!viewer->updatePointCloud(contour, color, name))
    //  viewer->addPointCloud (contour, color, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
//
//    plane->clear();
//	for(size_t j=0;j<inlier_indices[i].indices.size();j++)
//	{
//		plane->push_back(cloud->points[inlier_indices[i].indices[j]]);
//	}
//    sprintf (name, "planein_%02d", int (i));
//    pcl::visualization::PointCloudColorHandlerCustom <PointT> color1 (plane, red[i%6], grn[i%6], blu[i%6]);
//    if(!viewer->updatePointCloud(plane, color1, name))
//      viewer->addPointCloud (plane, color1, name);
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  

	  contour->clear();
	  for(size_t j=0;j<occluded_edge.indices.size();j++)
	  {
		// index to the point_cloud;
		size_t idx1=occluded_edge.indices[j];
		// index to the corresponding plane;
		size_t idx2=occluded_corresponding_plane.indices[j];
	//	std::cout<<idx1<<","<<idx2<<std::endl;

		if (idx2==i+1)
			contour->push_back(cloud->at(idx1));
	  }
	  std::cout<<"contour: "<<contour->size()<<std::endl;
		sprintf (name, "planein_%02d", int (i));
		pcl::visualization::PointCloudColorHandlerCustom <PointT> color1 (contour, red[i%6], grn[i%6], blu[i%6]);
		if(!viewer->updatePointCloud(contour, color1, name))
		  viewer->addPointCloud (contour, color1, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }

  	  int i=regions.size ();
	  contour->clear();
	  for(size_t j=0;j<occluded_edge.indices.size();j++)
	  {
		// index to the point_cloud;
		size_t idx1=occluded_edge.indices[j];
		// index to the corresponding plane;
		size_t idx2=occluded_corresponding_plane.indices[j];
	//	std::cout<<idx1<<","<<idx2<<std::endl;

		if (idx2==0)
			contour->push_back(cloud->at(idx1));
	  }
	  std::cout<<"contour: "<<contour->size()<<std::endl;
		sprintf (name, "planein_%02d", int (i));
		pcl::visualization::PointCloudColorHandlerCustom <PointT> color1 (contour, red[i%6], grn[i%6], blu[i%6]);
		if(!viewer->updatePointCloud(contour, color1, name))
		  viewer->addPointCloud (contour, color1, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
}

void
displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
					  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
					  std::vector<pcl::PointIndices> inlier_indices )
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>);

  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
	viewer->removeShape(name);
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
    
    //contour->points = regions[i].getContour ();
    //sprintf (name, "plane_%02d", int (i));
    //pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
    //if(!viewer->updatePointCloud(contour, color, name))
    //  viewer->addPointCloud (contour, color, name);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

    plane->clear();
	for(size_t j=0;j<inlier_indices[i].indices.size();j++)
	{
		plane->push_back(cloud->points[inlier_indices[i].indices[j]]);
	}
    sprintf (name, "planein_%02d", int (i));
    pcl::visualization::PointCloudColorHandlerCustom <PointT> color1 (plane, red[i%6], grn[i%6], blu[i%6]);
    if(!viewer->updatePointCloud(plane, color1, name))
      viewer->addPointCloud (plane, color1, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

//	pcl::ModelCoefficients coefficients;
//	coefficients.values.resize(4);
//	for(size_t j=0;j<4;j++)
//	{
//		coefficients.values[j]=regions[i].getCoefficients()(j);
//	}
//    sprintf (name, "pl_%02d", int (i));
//	viewer->removeShape(name);
//	viewer->addPlane(coefficients,centroid(0),centroid(1),centroid(2),name);
  }
}

void
displayEdges (std::vector<pcl::PointIndices> inlier_indices,
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
					  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
    occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
    nan_boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::copyPointCloud (*cloud, inlier_indices[0].indices, *nan_boundary_edges);
  pcl::copyPointCloud (*cloud, inlier_indices[1].indices, *occluding_edges);
  pcl::copyPointCloud (*cloud, inlier_indices[2].indices, *occluded_edges);
  pcl::copyPointCloud (*cloud, inlier_indices[3].indices, *high_curvature_edges);
  pcl::copyPointCloud (*cloud, inlier_indices[4].indices, *rgb_edges);

          if (!viewer->updatePointCloud<PointT> (nan_boundary_edges, "nan boundary edges"))
            viewer->addPointCloud<PointT> (nan_boundary_edges, "nan boundary edges");

          if (!viewer->updatePointCloud<PointT> (occluding_edges, "occluding edges"))
            viewer->addPointCloud<PointT> (occluding_edges, "occluding edges");
          
          if (!viewer->updatePointCloud<PointT> (occluded_edges, "occluded edges"))
            viewer->addPointCloud<PointT> (occluded_edges, "occluded edges");

          if (!viewer->updatePointCloud<PointT> (high_curvature_edges, "high curvature edges"))
            viewer->addPointCloud<PointT> (high_curvature_edges, "high curvature edges");

          if (!viewer->updatePointCloud<PointT> (rgb_edges, "rgb edges"))
            viewer->addPointCloud<PointT> (rgb_edges, "rgb edges");


  const int point_size = 2;
  //viewer->addPointCloud<pcl::PointXYZRGBA> (nan_boundary_edges, "nan boundary edges");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "nan boundary edges");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "nan boundary edges");

  //viewer->addPointCloud<pcl::PointXYZRGBA> (occluding_edges, "occluding edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluding edges");

  //viewer->addPointCloud<pcl::PointXYZRGBA> (occluded_edges, "occluded edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluded edges");

  //viewer->addPointCloud<pcl::PointXYZRGBA> (high_curvature_edges, "high curvature edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges");

  //viewer->addPointCloud<pcl::PointXYZRGBA> (rgb_edges, "rgb edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

}


void
displayCurvature (pcl::PointCloud<PointT>& cloud, pcl::PointCloud<pcl::Normal>& normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> curvature_cloud = cloud;
  for (size_t i  = 0; i < cloud.points.size (); i++)
  {
    if (normals.points[i].curvature < 0.04)
    {
      curvature_cloud.points[i].r = 0;
      curvature_cloud.points[i].g = 255;
      curvature_cloud.points[i].b = 0;
    }
    else
    {
      curvature_cloud.points[i].r = 255;
      curvature_cloud.points[i].g = 0;
      curvature_cloud.points[i].b = 0;
    }
  }
  
  if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature"))
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature");
  
}


