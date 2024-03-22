/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-06-08 09:41
#
# Filename:		data_reading.cpp
#
# Description: 
#
===============================================*/
#include "data_reading.h"

namespace ulysses
{
	// DataReading.
	// + input
	//   - path_name: path of the folder locating the rgb.txt, depth.txt, rgb folder and depth folder, etc.
	// + function
	//   - assignment of camera intrinsic parameters;
	//   - allocate the point_cloud;
	//   - set default StorageType as Time_Name;
	//   - set default debug as false;
	DataReading::DataReading(const std::string f): freiburg(f)
	{
		fx=517.3;//591.1;//567.6;//580.8;//525;
		fy=516.5;//590.1;//570.2;//581.8;//525;
		cx=318.6;//331;//324.7;//308.8;//319.5;
		cy=255.3;//234;//250.1;//253;//239.5;
		factor=5000;
		dx=9.3e-3;
		dy=9.3e-3;
		Width=640;
		Height=480;
		first_frame=true;
	}

	// setNormalEstimation
	// - set the integral image normal estimation method;
	// - and some necessary parameters;
	void DataReading::setNormalEstimation(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, 
										  pcl::Normal>::NormalEstimationMethod method, 
										  float MaxDepthChangeFactor, float NormalSmoothingSize)
	{
		normal_estimate_integral.setNormalEstimationMethod (method);
		normal_estimate_integral.setMaxDepthChangeFactor(MaxDepthChangeFactor);
		normal_estimate_integral.setNormalSmoothingSize(NormalSmoothingSize);
	}

	// Initialize
	// + input
	//   - time_start: read the image sequence from time_start.
	// + function
	//   - allocate space for scan;
	//   - open rgb.txt and depth.txt;
	//   - set timestamp to the start time;
	void DataReading::Initialize(double time_start)
	{
		std::string file_rgb=freiburg+"/rgb.txt";
		std::string file_depth=freiburg+"/depth.txt";
		std::string file_gt=freiburg+"/groundtruth.txt";
		// open the file rgb.txt and depth.txt;
		fp_rgb.open(file_rgb.c_str(),std::ios::in);
		fp_depth.open(file_depth.c_str(),std::ios::in);
		fp_gt.open(file_gt.c_str(),std::ios::in);
		std::string tmp_cache;

		// skip the comments started by character "#";
		// this implementation sucks;
		// needs modification;
		for(int i=0;i<9;i++)
		{
			fp_rgb>>tmp_cache;
			fp_depth>>tmp_cache;
		}
		for(int i=0;i<16;i++)
		{
			fp_gt>>tmp_cache;
		}
		if(time_start!=0)
		{
			// if the time_start is set by users,
			// then set the timestamp with the user-defined value;
			timestamp=time_start;
		}
		else
		{
			// if not, set the timestamp_rgb and timestamp_depth
			// to the first time stamp of the sequence;
			fp_rgb>>timestamp_rgb>>filename_rgb;
			fp_depth>>timestamp_depth>>filename_depth;
			// set timestamp to the larger one (subsequent one);
			if(timestamp_rgb>timestamp_depth)
				timestamp=timestamp_rgb;
			else
				timestamp=timestamp_depth;
		}

		if(debug)
		{
			std::cout<<"sample image sequence "<<freiburg<<" from "<<std::fixed<<timestamp<<std::endl;
		}
	}

	// SampleDataset
	// - sample the image sequence at next time step.
	void DataReading::SampleDataset()
	{
		// sample the images collected at timestamep;
		do
		{
			fp_rgb>>timestamp_rgb>>filename_rgb;
		}while(timestamp_rgb<timestamp);
		do
		{
			fp_depth>>timestamp_depth>>filename_depth;
		}while(timestamp_depth<timestamp);
		do
		{
			fp_gt>>timestamp_gt>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
		}while(timestamp_gt<timestamp);

		if(first_frame)
		{
			Eigen::Quaterniond quat_1(qw,qx,qy,qz);
			Eigen::Vector3d vec3d_1(tx,ty,tz);
			Tg=Transform(quat_1,vec3d_1);
			first_frame=false;
		}

		if(debug)
		{
			std::cout<<"timestamp: "<<std::fixed<<timestamp<<std::endl;
			std::cout<<"sample rgb image at "<<std::fixed<<timestamp_rgb<<","<<filename_rgb<<std::endl;
			std::cout<<"sample depth image at "<<std::fixed<<timestamp_depth<<","<<filename_depth<<std::endl;
			std::cout<<"sample groundtruth at "<<std::fixed<<timestamp_gt<<","<<tx<<" "<<ty<<" "<<tz<<", "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
		}

		// increase the timestamp by the sample_interval;
		timestamp+=sample_interval;
	}

	// loadScan
	// - load the rgb and depth images to point_cloud, normal_cloud and pixel_cloud;
	Scan* DataReading::loadScan()
	{
		Scan *scan=new Scan;
		scan->point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan->normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		scan->pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

		// sample the image sequence;
		SampleDataset();
		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;
		// full path of the current rgb and depth image;
		std::string filename_rgb_full=freiburg+"/"+filename_rgb;
		std::string filename_depth_full=freiburg+"/"+filename_depth;
		// load the rgb and depth image to cv::Mat;
		// the depth_image is stored as CV_8UC2;
		scan->img_rgb=cv::imread(filename_rgb_full);
		scan->img_depth=cv::imread(filename_depth_full,-1);

		// pointer to the Mat data;
		rgb_ptr=scan->img_rgb.data;
		depth_ptr=scan->img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		scan->point_cloud->clear();
		scan->normal_cloud->clear();
		scan->pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<scan->img_rgb.rows;i++)
		{
			for(int j=0;j<scan->img_rgb.cols;j++)
			{
				// 3 channels for one pixel in rgb image;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
				rgb_ptr++;
				// 2 channels for one pixel in depth image;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				point_tmp.z=*depth_tmp_ptr/factor;
				depth_ptr+=2;
				// transformation from pixel coordinate to the camera coordinate;
				// wrong results if considering length of the pixel;
				//point_tmp.x=(j-cx)*point_tmp.z*dx/fx;
				//point_tmp.y=(i-cy)*point_tmp.z*dy/fy;
				point_tmp.x=(j-cx)*point_tmp.z/fx;
				point_tmp.y=(i-cy)*point_tmp.z/fy;
				scan->point_cloud->push_back(point_tmp);
			}
		}
		// organize the point_cloud for the normal estimation;
		scan->point_cloud->width=Width;
		scan->point_cloud->height=Height;
		// generate the normal_cloud;
		normal_estimate_integral.setInputCloud(scan->point_cloud);
		normal_estimate_integral.compute (*scan->normal_cloud);
		// generate the pixel_cloud;
		for(int u=0;u<Height;u++)
		{
			for(int v=0;v<Width;v++)
			{
				tmp_pointxy.x=v;
				tmp_pointxy.y=u;
				scan->pixel_cloud->push_back(tmp_pointxy);
			}
		}

		Eigen::Quaterniond quat(qw,qx,qy,qz);
		Eigen::Vector3d vec3d(tx,ty,tz);
		Transform Tc(quat,vec3d);

//		if(debug)
//		{
//			std::cout<<"Tg:"<<Tg.R.determinant()<<std::endl<<Tg.getMatrix4f()<<std::endl;
//			std::cout<<"Tc:"<<Tc.R.determinant()<<std::endl<<Tc.getMatrix4f()<<std::endl;
//		}

		Tc=Tg.leftMul(Tc.inv());
		scan->Tcg_gt=new Transform(Tc);
		
//		if(debug)
//		{
//			std::cout<<"Tc:"<<std::endl<<Tc.getMatrix4f()<<std::endl;
//			std::cout<<"Tcg_gt:"<<std::endl<<scan->Tcg_gt->getMatrix4f()<<std::endl;
//		}

		return scan;
	}
}
