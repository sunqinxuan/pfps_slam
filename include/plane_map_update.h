/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-06-12 17:10
#
# Filename: plane_map_update.h
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
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <limits>
#include "ANN/ANN.h"
#include "types.h"

namespace ulysses
{
	class PlaneMapUpdate
	{
	public:

		PlaneMapUpdate() 
		{
			win_size=1;
		}
		~PlaneMapUpdate() {}

		// initialize
		// - 
		void initialize();

		void setDebug(bool d) {debug=d;}

		// updateMap
		// - update the plane map using existing map and the observed plane features;
		// - planes: observed plane features;
		// - map
		//   + planes
		//     - salient_point_seq
		//     - ptr_camera_pose_seq
		//   + camera_poses
		//     - ptr_planes
		void updateMap(Map *map, Scan *scan);//, std::vector<PlanePair> matched_planes);//, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		void clearMap(Map *map);


	private:
		
		bool debug;

		// win_size
		// - no. of frames the current plane salient points associates;
		int win_size;
		
		// mergePlanes
		void mergePlanes(Plane *plane, Plane *plane_scan);
	};
}

