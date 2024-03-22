/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-07-11 21:39
#
# Filename:		pose_estimation.cpp
#
# Description: 
#
===============================================*/

#include "pose_estimation.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ulysses
{

	void EdgeSE3pra::computeError()
	{
		const g2o::VertexSE3Expmap* v  =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
		Eigen::Vector3d p_local = v->estimate().map ( p_global );
		_error=(p_local-_measurement)/p_local(2);
		
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



	Transform PoseEstimation::alignPlanes(std::vector<PlanePair> matched_planes)
	{
		static const int N=matched_planes.size();
		ConstraintCase tmp_case=constraint_case(matched_planes);
		Eigen::Matrix3d tmp_mat3d,tmp_inverse;
		bool invertible;
		Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
		Eigen::MatrixXd A;
		Eigen::VectorXd d;
		if(tmp_case==DoF_6)
		{
			//Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
			A=Eigen::MatrixXd::Zero(N,3);
			d=Eigen::VectorXd::Zero(N);
			for(int i=0;i<N;i++)
			{
				A.block<1,3>(i,0)=matched_planes[i].ref->coefficients.block<3,1>(0,0).transpose();
				d(i)=matched_planes[i].cur->coefficients(3)-matched_planes[i].ref->coefficients(3);
			}
		}
		else if(tmp_case==DoF_5)
		{
			//Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
			if(abs(Tcr_align_planes.R.determinant()+1.0f)<1.0e-4)
			{
				H_svd_U.block<3,1>(0,2)=-H_svd_U.block<3,1>(0,2);
				Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
				if(debug)
				{
					std::cout<<"U':"<<std::endl<<H_svd_U<<std::endl;
					std::cout<<"det(R'):"<<Tcr_align_planes.R.determinant()<<std::endl;
				}
			}
			else
			{
				if(debug)
				{
					std::cout<<"U:"<<std::endl<<H_svd_U<<std::endl;
					std::cout<<"det(R):"<<Tcr_align_planes.R.determinant()<<std::endl;
				}
			}
			A=Eigen::MatrixXd::Zero(N+1,3);
			d=Eigen::VectorXd::Zero(N+1);
			for(int i=0;i<N;i++)
			{
				//if(i==1) continue;
				A.block<1,3>(i,0)=matched_planes[i].ref->coefficients.block<3,1>(0,0).transpose();
				d(i)=matched_planes[i].cur->coefficients(3)-matched_planes[i].ref->coefficients(3);
			}
			A.block<1,3>(N,0)=H_svd_V.block<3,1>(0,2).transpose();
		}
		else if(tmp_case==DoF_3)
		{
			Eigen::Matrix3d H1, H_svd_U1, H_svd_V1;
			H1=H+H_svd_U.block<3,1>(0,2)*H_svd_V.block<3,1>(0,2).transpose();
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H1, Eigen::ComputeFullU | Eigen::ComputeFullV);
			H_svd_U1=svd.matrixU();
			H_svd_V1=svd.matrixV();
			Tcr_align_planes.R=H_svd_U1*H_svd_V1.transpose();
			if(abs(Tcr_align_planes.R.determinant()+1.0f)<1.0e-4)
			{
				H_svd_U1.block<3,1>(0,2)=-H_svd_U1.block<3,1>(0,2);
				Tcr_align_planes.R=H_svd_U1*H_svd_V1.transpose();
				if(debug)
				{
					std::cout<<"U1':"<<std::endl<<H_svd_U1<<std::endl;
					std::cout<<"det(R'):"<<Tcr_align_planes.R.determinant()<<std::endl;
				}
			}
			else
			{
				if(debug)
				{
					std::cout<<"U1:"<<std::endl<<H_svd_U1<<std::endl;
					std::cout<<"det(R):"<<Tcr_align_planes.R.determinant()<<std::endl;
				}
			}
			A=Eigen::MatrixXd::Zero(N+2,3);
			d=Eigen::VectorXd::Zero(N+2);
			for(int i=0;i<N;i++)
			{
				A.block<1,3>(i,0)=matched_planes[i].ref->coefficients.block<3,1>(0,0).transpose();
				d(i)=matched_planes[i].ref->coefficients(3)-matched_planes[i].cur->coefficients(3);
			}
			A.block<1,3>(N,0)=H_svd_V.block<3,1>(0,1).transpose();
			A.block<1,3>(N+1,0)=H_svd_V.block<3,1>(0,2).transpose();
		}
		tmp_mat3d=A.transpose()*A;
		tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
		if(invertible)
			Tcr_align_planes.t=tmp_inverse*A.transpose()*d;
		else
		{
			std::cerr<<"matrix A uninvertible!"<<std::endl;
		}
		Tcr_align_planes.inverse();
		if(debug)
		{
			std::cout<<"constrain case:"<<tmp_case<<std::endl;
			std::cout<<"A"<<std::endl<<A<<std::endl;
			std::cout<<"d:"<<d.transpose()<<std::endl;
			std::cout<<"camera pose:"<<std::endl;
			std::cout<<"Rotation:"<<std::endl<<Tcr_align_planes.R<<std::endl;
			std::cout<<"translation:"<<Tcr_align_planes.t.transpose()<<std::endl;
			std::cout<<"eular angle:"<<Tcr_align_planes.Eulars().transpose()<<std::endl;
			std::cout<<"quatenion:"<<Tcr_align_planes.Quat().w()<<","<<Tcr_align_planes.Quat().vec().transpose()<<std::endl;
		}
		return Tcr_align_planes;
	}


	PoseEstimation::ConstraintCase PoseEstimation::constraint_case(std::vector<PlanePair> matched_planes)
	{
		H.setZero();
		for(int i=0;i<matched_planes.size();i++)
		{
			H=H+matched_planes[i].ref->coefficients.block<3,1>(0,0)*matched_planes[i].cur->coefficients.block<3,1>(0,0).transpose();
		}

//		Eigen::Vector3d H_singularValues;
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		H_svd_U=svd.matrixU();
		H_svd_V=svd.matrixV();
		H_singularValues=svd.singularValues();
		if(debug)
		{
			std::cout<<"svd of H: "<<H_singularValues.transpose()<<std::endl;
			std::cout<<H_svd_U<<std::endl;
			std::cout<<H_svd_V<<std::endl;
		}
		if(H_singularValues(0)>100*H_singularValues(1))
			return DoF_3;
		else if(H_singularValues(1)>100*H_singularValues(2))
			return DoF_5;
		else
			return DoF_6;
	}

	Eigen::Vector3d PoseEstimation::projectPoint2Plane(Eigen::Vector3d point, Eigen::Vector4d plane, bool vertical)
	{
		Eigen::Vector3d pro;
		Eigen::Vector3d n=plane.block<3,1>(0,0);
		if(vertical)
		{
			double t=(n.transpose()*point+plane(3))/(n.transpose()*n);
//			pro(0)=point(0)-n(0)*t;
//			pro(1)=point(1)-n(1)*t;
//			pro(2)=point(2)-n(2)*t;
			pro=point-n*t;
		}
		else
		{
			double mu=-plane(3)/(n.transpose()*point);
			pro=mu*point;
		}
		return pro;
	}

	Eigen::Vector3d PoseEstimation::projectPoint2Plane(Eigen::Vector3d point, Eigen::Vector4f plane)
	{
		Eigen::Vector4d pln;
		for(size_t i=0;i<4;i++)
		{
			pln(i)=(double)plane(i);
		}
		return projectPoint2Plane(point,pln);
	}

	Eigen::Matrix3d PoseEstimation::skewSym(Eigen::Vector3d p)
	{
		Eigen::Matrix3d m;
		double x=p(0);
		double y=p(1);
		double z=p(2);

		m(0,0)=0;
		m(1,1)=0;
		m(2,2)=0;

		m(0,1)=-z;
		m(1,0)=z;
		m(0,2)=y;
		m(2,0)=-y;
		m(1,2)=-x;
		m(2,1)=x;

		return m;
	}

	void PoseEstimation::align(Scan *scan, std::vector<PlanePair> matched_planes, Transform& Tcr,// IntrinsicParam int_param
								boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
/*
		// transform the occluding and occluded points on plane in ref frame to cur frame;
		// - occluding_points - Tcr(p^r_s);
		// - occluded_points  - Tcr(p^r_s+a^r_s), actually corresponding occluding points;
		// and then project them onto the image domain of cur frame;
		// - occluding_pixels
		// - occluded_pixels
		Eigen::Vector3d vec3d;
		std::vector<Eigen::Vector3d> occluding_points, occluded_points;
		std::vector<Eigen::Vector3d> occluding_pixels, occluded_pixels;
		for(size_t i=0;i<matched_planes.size();i++)
		{
			for(size_t j=0;j<matched_planes[i].ref->salient_points.size();j++)
			{
				vec3d(0)=matched_planes[i].ref->salient_points.at(j).x;
				vec3d(1)=matched_planes[i].ref->salient_points.at(j).y;
				vec3d(2)=matched_planes[i].ref->salient_points.at(j).z;
				if(vec3d.transpose()*matched_planes[i].ref->projective_rays[j]>0)
				{ //occluding;
					vec3d=Tcr.transform(vec3d);
					occluding_points.push_back(vec3d);
					vec3d=Tcr.intrinsic.project(vec3d);
					occluding_pixels.push_back(vec3d);
				}
				else if(vec3d.transpose()*matched_planes[i].ref->projective_rays[j]<0)
				{ //occluded;
					vec3d=Tcr.transform(vec3d+matched_planes[i].ref->projective_rays[j]);
					occluded_points.push_back(vec3d);
					vec3d=Tcr.intrinsic.project(vec3d);
					occluded_pixels.push_back(vec3d);
				}
				else //if(vec3d.transpose()*matched_planes[i].ref->projective_rays[j]=0)
				{ //high curvature;
				}
			}
		}

		{
			ofstream fp;
			fp.open("pixels.txt",std::ios::out);
			cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);

			for (size_t i=0;i<scan->edge_label_cloud->height;i++)
			{
				for (size_t j=0;j<scan->edge_label_cloud->width;j++)
				{
//					if(scan->edge_label_cloud->at(j,i).label==1)
//					{ // nan edge;
//						plane_img.at<cv::Vec3b>(i,j)[0]=255;
//						plane_img.at<cv::Vec3b>(i,j)[1]=0;
//						plane_img.at<cv::Vec3b>(i,j)[2]=0;
//					}
					if(scan->edge_label_cloud->at(j,i).label==2)
					{ // occluding;
						plane_img.at<cv::Vec3b>(i,j)[0]=255;
						plane_img.at<cv::Vec3b>(i,j)[1]=0;
						plane_img.at<cv::Vec3b>(i,j)[2]=0;
					}
					if(scan->edge_label_cloud->at(j,i).label==4)
					{ // occluded;
						plane_img.at<cv::Vec3b>(i,j)[0]=255;
						plane_img.at<cv::Vec3b>(i,j)[1]=255;
						plane_img.at<cv::Vec3b>(i,j)[2]=255;
					}
				}
			}


			for(size_t i=0;i<occluding_pixels.size();i++)
			{
				fp<<occluding_pixels[i].transpose()<<std::endl;
				int col=floor(occluding_pixels[i](0));
				int row=floor(occluding_pixels[i](1));
				plane_img.at<cv::Vec3b>(row,col)[0]=0;
				plane_img.at<cv::Vec3b>(row,col)[1]=255;
				plane_img.at<cv::Vec3b>(row,col)[2]=0;
			}
			fp<<"**********************************************************"<<std::endl;
			for(size_t i=0;i<occluded_pixels.size();i++)
			{
				fp<<occluded_pixels[i].transpose()<<std::endl;
				int col=floor(occluded_pixels[i](0));
				int row=floor(occluded_pixels[i](1));
				plane_img.at<cv::Vec3b>(row,col)[0]=0;
				plane_img.at<cv::Vec3b>(row,col)[1]=0;
				plane_img.at<cv::Vec3b>(row,col)[2]=255;
			}

			cv::imshow("img",plane_img);
			cv::waitKey(0);
			fp.close();
		}

		return;
*/

//		std::vector<Eigen::Vector3d> points;
////		Eigen::Vector3d vec3d;
//		for(size_t i=0;i<matched_planes.size();i++)
//		{
//			for(size_t j=0;j<matched_planes[i].cur->salient_points.size();j++)
//			{
//				vec3d(0)=matched_planes[i].cur->salient_points.at(j).x;
//				vec3d(1)=matched_planes[i].cur->salient_points.at(j).y;
//				vec3d(2)=matched_planes[i].cur->salient_points.at(j).z;
//				if(vec3d.transpose()*matched_planes[i].cur->projective_rays[j]<=0)
//					continue;
//				points.push_back(vec3d);
//			}
//		}
		
//		std::vector<int> indices;
//		std::vector<double> dist;
		char id[20];
		ofstream fp;
		fp.open("constraint.txt",std::ios::out);

		// g2o initialization;
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose( debug );

		// build the kdtree structure for the occluding edge points in current scan;
		// using the 3d coordinates of occluding edge points in camera frame;
		ANNkd_tree *kdtree;
		ANNpointArray cur_occluding;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray point_index;
		ANNdistArray distance;
		point_index=new ANNidx[1];
		distance=new ANNdist[1];

		int iterations = 0;
		bool converged = false;
		double chi2_pre=99999999999, chi2=99999999999;
		Eigen::Isometry3d Tcr_tmp = Eigen::Isometry3d::Identity();
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr spt (new pcl::PointCloud<pcl::PointXYZRGBA>);

			Eigen::Vector3d vec_3d_ref, vec_3d_ref_trans, vec_3d_cur, vec_3d_proj;
			Eigen::Vector3d nc, nr;
			double dc, dr;
			Eigen::Matrix<double,6,1> Eij_w;
			Eigen::Matrix<double,6,3> epsij_w;
			Eigen::Matrix<double,6,6> Psi_ij;
			std::vector<Eigen::Matrix<double,6,6> > Psi_i;
			Psi_i.resize(matched_planes.size());
			Eigen::Matrix<double,6,6> Psi;
			Eigen::Matrix<double,6,6> Psi_pln;


//		while(iterations<10 && !converged)
		{
			iterations++;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.R, Tcr.t) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );

			ConstraintCase tmp_case=constraint_case(matched_planes);
			if(tmp_case==DoF_6)
			{
				fp<<"constraint case --- DoF_6"<<std::endl<<std::endl;
			}
			else if(tmp_case==DoF_5)
			{
				fp<<"constraint case --- DoF_5"<<std::endl<<std::endl;
			}
			else
			{
				fp<<"constraint case --- DoF_3"<<std::endl<<std::endl;
			}

			fp<<"H_singularValues --- "<<std::endl<<H_singularValues.transpose()<<std::endl;
			fp<<"H_svd_U --- "<<std::endl<<H_svd_U<<std::endl;
			fp<<"H_svd_V --- "<<std::endl<<H_svd_V<<std::endl<<std::endl;

			Psi_pln.setZero();
			for(size_t i=0;i<matched_planes.size();i++)
			{
				nc=matched_planes[i].cur->coefficients.block<3,1>(0,0);
				dc=matched_planes[i].cur->coefficients(3);
				nr=matched_planes[i].ref->coefficients.block<3,1>(0,0);
				dr=matched_planes[i].ref->coefficients(3);

				Psi_pln.block<3,3>(0,0)=Psi_pln.block<3,3>(0,0)+(dr-dc)*(dr-dc)*nr*nr.transpose();
				Psi_pln.block<3,3>(3,3)=Psi_pln.block<3,3>(3,3)+nc.cross(nr)*nc.cross(nr).transpose();

				fp<<"matched plane --- "<<i<<" --- <"<<matched_planes[i].cur->index<<","<<matched_planes[i].ref->index<<">"<<std::endl;
				fp<<"cur - "<<matched_planes[i].cur->coefficients.transpose()<<", "<<matched_planes[i].cur->count<<std::endl;
				fp<<"ref - "<<matched_planes[i].ref->coefficients.transpose()<<", "<<matched_planes[i].ref->count<<std::endl;
			}
			fp<<std::endl;

			fp<<"Psi_pln --- "<<std::endl;
			fp<<Psi_pln<<std::endl<<std::endl;

			Eigen::EigenSolver<Eigen::Matrix<double,6,6> > es(Psi_pln);
			Eigen::Matrix<std::complex<double>,6,1> eigenvalues=es.eigenvalues();
			Eigen::Matrix<std::complex<double>,6,6> eigenvectors=es.eigenvectors();

			double lambda_pln_avg=0;
			for(size_t i=0;i<6;i++)
			{
				lambda_pln(i)=eigenvalues(i).real();
				lambda_pln_avg+=lambda_pln(i);
				if(lambda_pln(i)<0.0001)
					lambda_pln(i)=0.0001;
				for(size_t j=0;j<6;j++)
				{
					q_pln(i,j)=eigenvectors(i,j).real();
				}
			}
			lambda_pln_avg/=6;

			fp<<"Psi_pln eigen values"<<std::endl;
			fp<<eigenvalues.transpose()<<std::endl;
			fp<<"Psi_pln eigen vectors"<<std::endl;
			fp<<eigenvectors<<std::endl<<std::endl;


			for(size_t i=0;i<matched_planes.size();i++)
			{
				Psi_i[i].setZero();
				if(matched_planes[i].cur->occluded_points_seq[0].size()==0 || matched_planes[i].ref->occluded_points_seq[0].size()==0)
					continue;
				cur_occluding=annAllocPts(matched_planes[i].cur->occluded_points_seq[0].size(),3);
				for(int j=0;j<matched_planes[i].cur->occluded_points_seq[0].size();j++)
				{
					cur_occluding[j][0]=matched_planes[i].cur->occluded_points_seq[0][j]->xyz(0)
									   +matched_planes[i].cur->occluded_points_seq[0][j]->proj_ray(0);
					cur_occluding[j][1]=matched_planes[i].cur->occluded_points_seq[0][j]->xyz(1)
									   +matched_planes[i].cur->occluded_points_seq[0][j]->proj_ray(1);
					cur_occluding[j][2]=matched_planes[i].cur->occluded_points_seq[0][j]->xyz(2)
									   +matched_planes[i].cur->occluded_points_seq[0][j]->proj_ray(2);
				}
				kdtree=new ANNkd_tree(cur_occluding,matched_planes[i].cur->occluded_points_seq[0].size(),3);

				nc=matched_planes[i].cur->coefficients.block<3,1>(0,0);
				dc=matched_planes[i].cur->coefficients(3);
				nr=matched_planes[i].ref->coefficients.block<3,1>(0,0);
				dr=matched_planes[i].ref->coefficients(3);

				for(size_t j=0;j<matched_planes[i].ref->occluded_points_seq[0].size();j++)
				{
					// occluded point on ref plane;
					vec_3d_ref=matched_planes[i].ref->occluded_points_seq[0][j]->xyz;
					// corresponding occluding point;
					vec_3d_ref=vec_3d_ref+matched_planes[i].ref->occluded_points_seq[0][j]->proj_ray;
					// project the occluding point of ref plane into the cur scan;
					vec_3d_ref_trans=Tcr.R*vec_3d_ref+Tcr.t;
					// associate to the closest occluding point in cur scan;
					query_point[0]=vec_3d_ref_trans(0);
					query_point[1]=vec_3d_ref_trans(1);
					query_point[2]=vec_3d_ref_trans(2);
					kdtree->annkSearch(query_point,1,point_index,distance,0);
					vec_3d_cur=matched_planes[i].cur->occluded_points_seq[0][point_index[0]]->xyz;
//					indices.push_back(point_index[0]);
//					dist.push_back(distance[0]);
//
//					// corresponding nearest neighbor in current plane;
//					vec_3d_cur(0)=points[point_index[0]](0);
//					vec_3d_cur(1)=points[point_index[0]](1);
//					vec_3d_cur(2)=points[point_index[0]](2);

					vec_3d_proj=projectPoint2Plane(vec_3d_ref_trans,matched_planes[i].cur->coefficients);
//					vec_3d_cur=matched_planes[i].cur->projectPoint2Plane(vec_3d_ref);

//					vec_3d_pixel=Tcr.intrinsic.project(vec_3d_cur);
//					scan->plane_label_cloud->at(floor(vec_3d_pixel(0)),floor(vec_3d_pixel(1)))

					double np=nc.transpose()*vec_3d_ref_trans;
					Eigen::Matrix3d vec_3d_ref_skewsym=skewSym(vec_3d_ref);
					epsij_w.block<3,3>(0,0)=np*Eigen::Matrix3d::Identity()-nc*vec_3d_ref_trans.transpose();
					epsij_w.block<3,3>(3,0)=np*vec_3d_ref_skewsym-vec_3d_ref_skewsym*nc*vec_3d_ref_trans.transpose();
					Eij_w=epsij_w*(vec_3d_cur-vec_3d_proj)*dc*2/(np*np);
					double alpha=0,weight=0;
					int dir;
					for(int k=0;k<6;k++)
					{
						alpha=fabs(Eij_w.transpose()*q_pln.block<6,1>(0,k));
						alpha=alpha/Eij_w.norm();
//						alpha/=sqrt(lambda_pln(dir)/lambda_pln_avg);
//						weight+=alpha;
						if(weight<alpha)
						{
							weight=alpha;
							dir=k;
						}
					}
					weight=weight/sqrt(lambda_pln(dir)/lambda_pln_avg);
//					weight/=6;
					fp<<Eij_w.transpose()<<", "<<dir<<", ";//<<weight<<", ";
					fp<<weight<<std::endl;
					Psi_ij=Eij_w*Eij_w.transpose()*weight;
					Psi_i[i]=Psi_i[i]+Psi_ij;

//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_proj(0);
//					pt1.y=vec_3d_proj(1);
//					pt1.z=vec_3d_proj(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);

					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_proj);//p_global
					edge->setVertex(0,pose);
					edge->setMeasurement(vec_3d_cur);
					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
					edge->setId(i);

					optimizer.addEdge(edge);
				}
				Psi_i[i]=Psi_i[i]/matched_planes[i].ref->occluded_points_seq[0].size();
				fp<<"matched plane --- "<<i<<" --- <"<<matched_planes[i].cur->index<<","<<matched_planes[i].ref->index<<">"<<std::endl;
				fp<<Psi_i[i]<<std::endl<<std::endl;

				Eigen::EigenSolver<Eigen::Matrix<double,6,6> > es(Psi_i[i]);
				Eigen::Matrix<std::complex<double>,6,1> eigenvalues=es.eigenvalues();
				Eigen::Matrix<std::complex<double>,6,6> eigenvectors=es.eigenvectors();

				fp<<"eigen values"<<std::endl;
				fp<<eigenvalues.transpose()<<std::endl;
				fp<<"eigen vectors"<<std::endl;
				fp<<eigenvectors<<std::endl<<std::endl;

				delete kdtree;
				annDeallocPts(cur_occluding);
			}

			Psi.setZero();
			for(size_t i=0;i<Psi_i.size();i++)
			{
				Psi=Psi+Psi_i[i];
			}
			Psi=Psi/Psi_i.size();
			fp<<"Psi --- "<<std::endl;
			fp<<Psi<<std::endl<<std::endl;

//			Eigen::EigenSolver<Eigen::Matrix<double,6,6> > es(Psi);
//			Eigen::Matrix<std::complex<double>,6,1> eigenvalues=es.eigenvalues();
//			Eigen::Matrix<std::complex<double>,6,6> eigenvectors=es.eigenvectors();
			es.compute(Psi);
			eigenvalues=es.eigenvalues();
			eigenvectors=es.eigenvectors();

			fp<<"Psi eigen values"<<std::endl;
			fp<<eigenvalues.transpose()<<std::endl;
			fp<<"Psi eigen vectors"<<std::endl;
			fp<<eigenvectors<<std::endl<<std::endl;


			
			optimizer.initializeOptimization();
			int result=optimizer.optimize ( 30 );
			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
//				break;
			}

			Tcr_tmp = pose->estimate();
//			Tcr=Tcr_tmp*Tcr;
			Tcr.leftMultiply(Tcr_tmp);

			// *****************************************************************************************
//			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBA>);
//			pcl::PointXYZRGBA tmp_point;
//			vis->removeAllPointClouds();
//			Eigen::Matrix4f trans;
//			for(size_t i=0;i<4;i++)
//			{
//				for(size_t j=0;j<4;j++)
//				{
//					trans(i,j)=Tcr.matrix()(i,j);
//				}
//			}
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

		scan->Tcg->R=Tcr.R;
		scan->Tcg->t=Tcr.t;

		delete point_index;
		delete distance;
		fp.close();
	}

	void PoseEstimation::align(Scan *scan, Scan *scan_ref, Transform& Tcr,
								boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		ofstream fp;
		fp.open("constraint.txt",std::ios::out);
		ofstream fpp;
		fpp.open("optimize.txt",std::ios::out);

		// g2o initialization;
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose( debug );

		std::vector<PlanePair> matched_planes;
		// build the kdtree structure for the occluding edge points in current scan;
		// using the 3d coordinates of occluding edge points in camera frame;
		ANNkd_tree *kdtree;
		ANNpointArray cur_occluding;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray point_index;
		ANNdistArray distance;
		point_index=new ANNidx[1];
		distance=new ANNdist[1];

		int iterations = 0;
		bool converged = false;
		double chi2_pre=99999999999, chi2=99999999999;
		Eigen::Isometry3d Tcr_tmp = Eigen::Isometry3d::Identity();
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr spt (new pcl::PointCloud<pcl::PointXYZRGBA>);

		Eigen::Vector3d vec_3d_ref, vec_3d_cur, vec_3d_pln;
		Eigen::Vector3d nc, nr;
		double dc, dr;
		Eigen::Matrix<double,6,6> Psi_pln;
		Eigen::Matrix<double,6,1> Jpk_w;
		Eigen::Matrix<double,6,6> Psi_k;
		Eigen::Matrix<double,6,6> Psi;

		fp<<"planes in scna_ref --- "<<std::endl<<std::endl;
		for(size_t i=0;i<scan_ref->observed_planes.size();i++)
		{
			fp<<"plane - "<<scan_ref->observed_planes[i]->index<<std::endl;
			fp<<"point quantity - "<<scan_ref->observed_planes[i]->count<<std::endl;
			fp<<"coefficients - "<<scan_ref->observed_planes[i]->coefficients.transpose()<<std::endl;
			fp<<"centroid - "<<scan_ref->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
			fp<<"covariance - "<<std::endl<<scan_ref->observed_planes[i]->covariance_xyz<<std::endl;
		}
		fp<<std::endl;
		fp<<"planes in scna_cur --- "<<std::endl<<std::endl;
		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			fp<<"plane - "<<scan->observed_planes[i]->index<<std::endl;
			fp<<"point quantity - "<<scan->observed_planes[i]->count<<std::endl;
			fp<<"coefficients - "<<scan->observed_planes[i]->coefficients.transpose()<<std::endl;
			fp<<"centroid - "<<scan->observed_planes[i]->centroid_xyz.transpose()<<std::endl;
			fp<<"covariance - "<<std::endl<<scan->observed_planes[i]->covariance_xyz<<std::endl;
		}
		fp<<std::endl;

		while(iterations<10 && !converged)
		{
			iterations++;
			fp<<"iteration "<<iterations<<"*************************************************"<<std::endl;
			fpp<<"iteration "<<iterations<<"*************************************************"<<std::endl;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.R, Tcr.t) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );

			fpp<<"vertex: "<<std::endl;
			fpp<<Tcr.R<<std::endl<<Tcr.t<<std::endl<<std::endl;


			matched_planes.clear();
			double sim_min=9999999;
			int idx_min;
//			std::vector<int> cur_idx;
//			cur_idx.clear();
			fp<<"matching planes ..."<<std::endl;
			for(size_t i=0;i<scan_ref->observed_planes.size();i++)
			{
				sim_min=9999999;
				idx_min=-1;
				Eigen::Vector3d centroid=Tcr.R*scan_ref->observed_planes[i]->centroid_xyz+Tcr.t;
				Eigen::Matrix3d covariance=Tcr.R*scan_ref->observed_planes[i]->covariance_xyz*Tcr.R.transpose();
				for(size_t j=0;j<scan->observed_planes.size();j++)
				{
//					std::cout<<"<"<<i<<","<<j<<"> "<<std::endl;
					double sim=scan->observed_planes[j]->similarity_geom(centroid,covariance);
					fp<<"<"<<i<<","<<j<<"> - "<<sim<<std::endl;
					if(sim<sim_min && scan->observed_planes[j]->normal.transpose()*Tcr.R*scan_ref->observed_planes[i]->normal>0.9)
					{
						sim_min=sim;
						if(sim>0.5)
							continue;
						idx_min=j;
//						cur_idx.push_back(j);
					}
				}
				if(idx_min==-1)
					continue;
				PlanePair plane_pair(scan_ref->observed_planes[i],scan->observed_planes[idx_min]);
				matched_planes.push_back(plane_pair);
			}

			for(size_t i=0;i<matched_planes.size();i++)
			{
				fp<<"matched plane --- "<<i<<" --- <"<<matched_planes[i].ref->index<<","<<matched_planes[i].cur->index<<">"<<std::endl;
//				fp<<"ref - "<<matched_planes[i].ref->coefficients.transpose()<<", "<<matched_planes[i].ref->count<<std::endl;
//				fp<<"cur - "<<matched_planes[i].cur->coefficients.transpose()<<", "<<matched_planes[i].cur->count<<std::endl;
//
//				fp<<"ref - "<<matched_planes[i].ref->centroid_xyz.transpose()<<std::endl;
//				fp<<matched_planes[i].ref->covariance_xyz<<std::endl;
//				fp<<"cur - "<<matched_planes[i].cur->centroid_xyz.transpose()<<std::endl;
//				fp<<matched_planes[i].cur->covariance_xyz<<std::endl;
			}
			fp<<std::endl;

			ConstraintCase tmp_case=constraint_case(matched_planes);
			if(tmp_case==DoF_6)
			{
				fp<<"constraint case --- DoF_6"<<std::endl<<std::endl;
			}
			else if(tmp_case==DoF_5)
			{
				fp<<"constraint case --- DoF_5"<<std::endl<<std::endl;
			}
			else
			{
				fp<<"constraint case --- DoF_3"<<std::endl<<std::endl;
			}

			fp<<"H_singularValues --- "<<std::endl<<H_singularValues.transpose()<<std::endl;
			fp<<"H_svd_U --- "<<std::endl<<H_svd_U<<std::endl;
			fp<<"H_svd_V --- "<<std::endl<<H_svd_V<<std::endl<<std::endl;

			Psi_pln.setZero();
			for(size_t i=0;i<matched_planes.size();i++)
			{
				nc=matched_planes[i].cur->coefficients.block<3,1>(0,0);
				dc=matched_planes[i].cur->coefficients(3);
				nr=matched_planes[i].ref->coefficients.block<3,1>(0,0);
				dr=matched_planes[i].ref->coefficients(3);

				Psi_pln.block<3,3>(0,0)=Psi_pln.block<3,3>(0,0)+(dr-dc)*(dr-dc)*nr*nr.transpose();
				Psi_pln.block<3,3>(3,3)=Psi_pln.block<3,3>(3,3)+nc.cross(nr)*nc.cross(nr).transpose();

				vec_3d_pln=Tcr.R*matched_planes[i].ref->centroid_xyz+Tcr.t;
				EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_pln);//p_global
				edge->setVertex(0,pose);
				edge->setMeasurement(matched_planes[i].cur->centroid_xyz);
				Eigen::Matrix3d inf=matched_planes[i].cur->normal*matched_planes[i].cur->normal.transpose();
				edge->setInformation(inf);
				edge->setId(i*2);
				optimizer.addEdge(edge);

//				EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_pln);//p_global
//				edge->setVertex(0,pose);
//				edge->setMeasurement(matched_planes[i].cur->centroid_xyz);
				inf=Tcr.R*matched_planes[i].ref->normal*matched_planes[i].ref->normal.transpose()*Tcr.R.transpose();
				edge->setInformation(inf);
				edge->setId(i*2+1);
				optimizer.addEdge(edge);

				fpp<<"matched plane --- "<<i<<" --- <"<<matched_planes[i].ref->index<<","<<matched_planes[i].cur->index<<">"<<std::endl;
				Eigen::Vector3d error=(matched_planes[i].cur->centroid_xyz-Tcr.R*matched_planes[i].ref->centroid_xyz-Tcr.t);
				fpp<<"error - "<<error.transpose()*inf*error<<std::endl;
			}
			fpp<<std::endl;

			fp<<"Psi_pln --- "<<std::endl;
			fp<<Psi_pln<<std::endl<<std::endl;

			Eigen::EigenSolver<Eigen::Matrix<double,6,6> > es(Psi_pln);
			Eigen::Matrix<std::complex<double>,6,1> eigenvalues=es.eigenvalues();
			Eigen::Matrix<std::complex<double>,6,6> eigenvectors=es.eigenvectors();

			double lambda_pln_avg=0;
			for(size_t i=0;i<6;i++)
			{
				lambda_pln(i)=eigenvalues(i).real();
				lambda_pln_avg+=lambda_pln(i);
				if(lambda_pln(i)<0.0001)
					lambda_pln(i)=0.0001;
				for(size_t j=0;j<6;j++)
				{
					q_pln(i,j)=eigenvectors(i,j).real();
				}
			}
			lambda_pln_avg/=6;

			fp<<"Psi_pln eigen values"<<std::endl;
			fp<<lambda_pln.transpose()<<std::endl;
			fp<<"Psi_pln eigen vectors"<<std::endl;
			fp<<q_pln<<std::endl<<std::endl;

			// build the kdtree using all the occluding edge points in cur scan;
			cur_occluding=annAllocPts(scan->edge_indices[1].indices.size(),3);
			for(size_t i=0;i<scan->edge_indices[1].indices.size();i++)
			{
				int idx=scan->edge_indices[1].indices[i];
				cur_occluding[i][0]=scan->point_cloud->at(idx).x;
				cur_occluding[i][1]=scan->point_cloud->at(idx).y;
				cur_occluding[i][2]=scan->point_cloud->at(idx).z;
			}
			kdtree=new ANNkd_tree(cur_occluding,scan->edge_indices[1].indices.size(),3);

			Psi.setZero();
			for(size_t i=0;i<scan_ref->edge_indices[1].indices.size();i++)
			{
				int idx=scan_ref->edge_indices[1].indices[i];
				vec_3d_ref(0)=scan_ref->point_cloud->at(idx).x;
				vec_3d_ref(1)=scan_ref->point_cloud->at(idx).y;
				vec_3d_ref(2)=scan_ref->point_cloud->at(idx).z;
				vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
				query_point[0]=vec_3d_ref(0);
				query_point[1]=vec_3d_ref(1);
				query_point[2]=vec_3d_ref(2);
				kdtree->annkSearch(query_point,1,point_index,distance,0);
				int idx_cur=scan->edge_indices[1].indices[point_index[0]];
				vec_3d_cur(0)=scan->point_cloud->at(idx_cur).x;
				vec_3d_cur(1)=scan->point_cloud->at(idx_cur).y;
				vec_3d_cur(2)=scan->point_cloud->at(idx_cur).z;

				Eigen::Matrix3d vec_3d_ref_skewsym=skewSym(vec_3d_ref);
				Jpk_w.block<3,1>(0,0)=-2*(vec_3d_cur-vec_3d_ref);
				Jpk_w.block<3,1>(3,0)=-2*vec_3d_ref_skewsym*(vec_3d_cur-vec_3d_ref);
				double alpha=0,weight=0;
				int dir;
				for(int k=0;k<6;k++)
				{
					alpha=fabs(Jpk_w.transpose()*q_pln.block<6,1>(0,k));
					alpha=alpha/Jpk_w.norm();
					if(weight<alpha)
					{
						weight=alpha;
						dir=k;
					}
				}
				weight=weight/sqrt(lambda_pln(dir)/lambda_pln_avg);
				fp<<Jpk_w.transpose()<<", "<<dir<<", ";//<<weight<<", ";
				fp<<weight<<std::endl;
				Psi_k=Jpk_w*Jpk_w.transpose()*weight;
				Psi=Psi+Psi_k;

				EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
				edge->setVertex(0,pose);
				edge->setMeasurement(vec_3d_cur);
				edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
				edge->setId(matched_planes.size()+i);
//				optimizer.addEdge(edge);

				fpp<<"point --- "<<i<<std::endl;
				fpp<<"cur - "<<vec_3d_cur.transpose()<<std::endl;
				fpp<<"ref - "<<vec_3d_ref.transpose()<<std::endl;
				Eigen::Vector3d error=vec_3d_cur-vec_3d_ref;
				fpp<<"error - "<<error.transpose()*error<<std::endl;
			}
			delete kdtree;
			annDeallocPts(cur_occluding);

			fp<<"Psi --- "<<std::endl;
			fp<<Psi<<std::endl<<std::endl;

			es.compute(Psi);
			eigenvalues=es.eigenvalues();
			eigenvectors=es.eigenvectors();

			fp<<"Psi eigen values"<<std::endl;
			fp<<eigenvalues.transpose()<<std::endl;
			fp<<"Psi eigen vectors"<<std::endl;
			fp<<eigenvectors<<std::endl<<std::endl;


			
			optimizer.initializeOptimization();
			int result=optimizer.optimize ( 30 );
			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
				break;
			}

			Tcr_tmp = pose->estimate();
			Tcr.leftMultiply(Tcr_tmp);

			// *****************************************************************************************
//			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBA>);
//			pcl::PointXYZRGBA tmp_point;
//			vis->removeAllPointClouds();
//			Eigen::Matrix4f trans;
//			for(size_t i=0;i<4;i++)
//			{
//				for(size_t j=0;j<4;j++)
//				{
//					trans(i,j)=Tcr.matrix()(i,j);
//				}
//			}
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

		scan->Tcg->R=Tcr.R;
		scan->Tcg->t=Tcr.t;

		delete point_index;
		delete distance;
		fp.close();
	}


	void PoseEstimation::align(Scan *scan, Scan *scan_ref, Transform& Tcr)
	{
		ofstream fp;
		fp.open("pose_estimation.txt",std::ios::out);
		timeval start, end;
		double timeused;

		std::vector<int> indices;
		std::vector<double> dist;
		char id[20];
		Eigen::Vector3d vec_3d_ref, vec_3d_cur;
		Eigen::Vector3d vec_3d_pixel;
		Eigen::Vector4d vec_4d_plane, vec_4d_point;

		// g2o initialization;
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose( debug );

		// build the kdtree structure for the occluding edge points in current scan;
		// using the 3d coordinates of occluding edge points in camera frame;
		ANNkd_tree *kdtree_occluding, *kdtree_occluded;
		ANNpointArray occluding_cur, occluded_cur;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray point_index=new ANNidx[1];
		ANNdistArray distance=new ANNdist[1];
		// all the occluding points in cur scan;
		occluding_cur=annAllocPts(scan->edge_indices[1].indices.size(),3);
		for(size_t i=0;i<scan->edge_indices[1].indices.size();i++)
		{
			int idx=scan->edge_indices[1].indices[i];
			occluding_cur[i][0]=scan->point_cloud->at(idx).x;
			occluding_cur[i][1]=scan->point_cloud->at(idx).y;
			occluding_cur[i][2]=scan->point_cloud->at(idx).z;
		}
		kdtree_occluding=new ANNkd_tree(occluding_cur,scan->edge_indices[1].indices.size(),3);
		// all the occluded points in cur scan;
		occluded_cur=annAllocPts(scan->edge_indices[2].indices.size(),3);
		for(size_t i=0;i<scan->edge_indices[2].indices.size();i++)
		{
			int idx=scan->edge_indices[2].indices[i];
			occluded_cur[i][0]=scan->point_cloud->at(idx).x;
			occluded_cur[i][1]=scan->point_cloud->at(idx).y;
			occluded_cur[i][2]=scan->point_cloud->at(idx).z;
		}
		kdtree_occluded=new ANNkd_tree(occluded_cur,scan->edge_indices[2].indices.size(),3);

		int iterations = 0;
		bool converged = false;
		double chi2_pre=99999999999, chi2=99999999999;
		Eigen::Isometry3d Tcr_tmp = Eigen::Isometry3d::Identity();


		cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
		std::vector<SalientPoint*> points;
		std::vector<SalientPoint*> occluding, occluded;


		iterations=0;
		while(iterations<max_iter_icp && !converged)
		{
//		    vis->removeAllShapes();
//			fp<<std::endl<<iterations<<"**************************************"<<std::endl;
			if(debug)
				std::cout<<std::endl<<"icp iteration --- "<<iterations<<"**************************************"<<std::endl;
			iterations++;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.R, Tcr.t) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );

			gettimeofday(&start,NULL);
			indices.clear();
			for(size_t i=0;i<scan_ref->observed_planes.size();i++)
			{
				size_t N=scan_ref->observed_planes[i]->occluding_points_seq.size();
				occluding.clear();
				occluding.insert(occluding.end(),scan_ref->observed_planes[i]->occluding_points_seq[N-1].begin(),
											     scan_ref->observed_planes[i]->occluding_points_seq[N-1].end());
				for(size_t j=0;j<occluding.size();j++)
				{
					vec_3d_ref=occluding[j]->xyz;
					// project the occluding point into the cur scan;
					vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
					// set the occluding point as the  query point for the kdtree;
					query_point[0]=vec_3d_ref(0);
					query_point[1]=vec_3d_ref(1);
					query_point[2]=vec_3d_ref(2);
					kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
					if(distance[0]>0.03)
						continue;

					// corresponding nearest neighbor in current plane;
					int idx=scan->edge_indices[1].indices[point_index[0]];
					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
					vec_3d_cur(2)=scan->point_cloud->at(idx).z;

					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
					edge->setVertex(0,pose);
					edge->setMeasurement(vec_3d_cur);
					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
					edge->setId(j*3);
					optimizer.addEdge(edge);
				}

				N=scan_ref->observed_planes[i]->occluded_points_seq.size();
				occluded.clear();
				occluded.insert(occluded.end(),scan_ref->observed_planes[i]->occluded_points_seq[N-1].begin(),
											   scan_ref->observed_planes[i]->occluded_points_seq[N-1].end());
				for(size_t j=0;j<occluded.size();j++)
				{
					vec_3d_ref=occluded[j]->xyz+occluded[j]->proj_ray;
					vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
					query_point[0]=vec_3d_ref(0);
					query_point[1]=vec_3d_ref(1);
					query_point[2]=vec_3d_ref(2);
					kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
					if(distance[0]>0.03)
						continue;

					// corresponding nearest neighbor in current plane;
					int idx=scan->edge_indices[1].indices[point_index[0]];
					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
					vec_3d_cur(2)=scan->point_cloud->at(idx).z;

					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
					edge->setVertex(0,pose);
					edge->setMeasurement(vec_3d_cur);
					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
					edge->setId(j*3+1);
					optimizer.addEdge(edge);

					// corresponding occluded point in current scan is on plane [idx_plane];
					if(scan->occluding_correspondences.indices[point_index[0]]==-1)
						continue;
					int idx_plane=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
					if(idx_plane>-1)
					{
						Eigen::Vector4f coeff=scan->planar_regions[idx_plane].getCoefficients();
						Eigen::Vector3d vec_3d_ref_proj=projectPoint2Plane(vec_3d_ref,coeff);
						query_point[0]=vec_3d_ref_proj(0);
						query_point[1]=vec_3d_ref_proj(1);
						query_point[2]=vec_3d_ref_proj(2);
						kdtree_occluded->annkSearch(query_point,1,point_index,distance,0);
						if(distance[0]>0.03)
							continue;
//						fp<<point_index[0]<<"\t"<<distance[0]<<std::endl;

						int idx=scan->edge_indices[2].indices[point_index[0]];
//						int idx=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
						vec_3d_cur(0)=scan->point_cloud->at(idx).x;
						vec_3d_cur(1)=scan->point_cloud->at(idx).y;
						vec_3d_cur(2)=scan->point_cloud->at(idx).z;

						EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref_proj);//p_global
						edge->setVertex(0,pose);
						edge->setMeasurement(vec_3d_cur);
						edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
						edge->setId(j*3+2);
						optimizer.addEdge(edge);
					}
				}

				if(debug)
				{
					std::cout<<"pose estimating: "<<i<<"th plane in the ref scan!"<<std::endl;
					std::cout<<"occluding_points_seq[0]: "<<occluding.size()<<std::endl;
					std::cout<<"occluded_points_seq[0]: "<<occluded.size()<<std::endl;
				}

			}
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

			if(debug)
			{
				std::cout<<"time adding edges:"<<timeused<<std::endl;
				std::cout<<"edge size - "<<optimizer.edges().size()<<std::endl;
			}

			gettimeofday(&start,NULL);
			optimizer.initializeOptimization();
			int result=optimizer.optimize ( max_iter_g2o );
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			if(debug)
				std::cout<<"time LM optimizing:"<<timeused<<std::endl;

			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
				break;
			}

			Tcr_tmp = pose->estimate();
			Tcr.leftMultiply(Tcr_tmp);
		}



		// so Tcg=Tcr;
		scan->Tcg->R=Tcr.R;
		scan->Tcg->t=Tcr.t;

		delete point_index;
		delete distance;
		delete kdtree_occluding;
		delete kdtree_occluded;
		annDeallocPt(query_point);
		annDeallocPts(occluding_cur);
		annDeallocPts(occluded_cur);
		fp.close();
	}


	void PoseEstimation::align(Scan *scan, Map *map, Transform& Tcr,// IntrinsicParam int_param
								boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		ofstream fp;
		fp.open("pose_estimation.txt",std::ios::out);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cur(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZRGBA>);
		timeval start, end;
		double timeused;

		std::vector<int> indices;
		std::vector<double> dist;
		char id[20];
		Eigen::Vector3d vec_3d_ref, vec_3d_cur;
		Eigen::Vector3d vec_3d_pixel;
		Eigen::Vector4d vec_4d_plane, vec_4d_point;

		// g2o initialization;
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose( debug );

		// build the kdtree structure for the occluding edge points in current scan;
		// using the 3d coordinates of occluding edge points in camera frame;
		ANNkd_tree *kdtree_occluding, *kdtree_occluded;
		ANNpointArray occluding_cur, occluded_cur;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray point_index=new ANNidx[1];
		ANNdistArray distance=new ANNdist[1];
		// all the occluding points in cur scan;
		occluding_cur=annAllocPts(scan->edge_indices[1].indices.size(),3);
		for(size_t i=0;i<scan->edge_indices[1].indices.size();i++)
		{
			int idx=scan->edge_indices[1].indices[i];
			occluding_cur[i][0]=scan->point_cloud->at(idx).x;
			occluding_cur[i][1]=scan->point_cloud->at(idx).y;
			occluding_cur[i][2]=scan->point_cloud->at(idx).z;
		}
		kdtree_occluding=new ANNkd_tree(occluding_cur,scan->edge_indices[1].indices.size(),3);
		// all the occluded points in cur scan;
		occluded_cur=annAllocPts(scan->edge_indices[2].indices.size(),3);
		for(size_t i=0;i<scan->edge_indices[2].indices.size();i++)
		{
			int idx=scan->edge_indices[2].indices[i];
			occluded_cur[i][0]=scan->point_cloud->at(idx).x;
			occluded_cur[i][1]=scan->point_cloud->at(idx).y;
			occluded_cur[i][2]=scan->point_cloud->at(idx).z;
		}
		kdtree_occluded=new ANNkd_tree(occluded_cur,scan->edge_indices[2].indices.size(),3);

		int iterations = 0;
		bool converged = false;
		double chi2_pre=99999999999, chi2=99999999999;
		Eigen::Isometry3d Tcr_tmp = Eigen::Isometry3d::Identity();


		cv::Mat plane_img=cv::Mat::zeros(480,640,CV_8UC3);
		std::vector<SalientPoint*> points;
		std::vector<SalientPoint*> occluding, occluded;
/*
		while(iterations<10 && !converged)
		{
		    vis->removeAllShapes();
			fp<<std::endl<<iterations<<"**************************************"<<std::endl;
			std::cout<<std::endl<<"icp iteration --- "<<iterations<<"**************************************"<<std::endl;
			iterations++;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.R, Tcr.t) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );

			indices.clear();
			for(size_t i=0;i<map->planes.size();i++)
			{
				// which planes in map is gonna used to align the current scan;
				if(!inViewFrustum(map,Tcr))
					continue;
				// points - salient points from map that is to align current observation;
				points.clear();
				points.insert(points.end(),map->planes[i]->occluding_points_seq[0].begin(),map->planes[i]->occluding_points_seq[0].end());
				points.insert(points.end(),map->planes[i]->occluded_points_seq[0].begin(),map->planes[i]->occluded_points_seq[0].end());

				if(debug)
				{
					std::cout<<"pose estimating: "<<i<<"th plane in the map!"<<std::endl;
//					std::cout<<"occluding_points: "<<map->planes[i]->occluding_points.size()<<std::endl;
//					std::cout<<"occluding_points_plane: "<<map->planes[i]->occluding_points_plane.size()<<std::endl;
					std::cout<<"occluding_points_seq[0]: "<<map->planes[i]->occluding_points_seq[0].size()<<std::endl;
					std::cout<<"occluded_points_seq[0]: "<<map->planes[i]->occluded_points_seq[0].size()<<std::endl;
					std::cout<<"points: "<<points.size()<<std::endl;
				}

				// the map is generated by only one observation;
				for(size_t j=0;j<points.size();j++)
				{
					// for one salient point in planes[i];
					if(points[j]->type==SalientPoint::TYPE::OCCLUDING)
					{ // occluding;
						vec_3d_ref=points[j]->xyz;
						// project the occluding point into the cur scan;
						vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
						// set the occluding point as the  query point for the kdtree;
						query_point[0]=vec_3d_ref(0);
						query_point[1]=vec_3d_ref(1);
						query_point[2]=vec_3d_ref(2);
						kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
						if(distance[0]>0.1)
							continue;
//						indices.push_back(point_index[0]);
//						dist.push_back(distance[0]);
//						fp<<std::endl<<point_index[0]<<"\t"<<distance[0]<<std::endl;
	
						// corresponding nearest neighbor in current plane;
						int idx=scan->edge_indices[1].indices[point_index[0]];
						int idx_pln=scan->edge_corresponding_plane[1].indices[point_index[0]];
						if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
							continue;
//						vec_3d_cur(0)=scan->point_cloud->at(idx).x;
//						vec_3d_cur(1)=scan->point_cloud->at(idx).y;
//						vec_3d_cur(2)=scan->point_cloud->at(idx).z;
						vec_3d_cur=projectPoint2Plane(vec_3d_ref,scan->observed_planes[idx_pln]->coefficients,true);

//						double min=9999999;
//						Eigen::Vector3d vec2plane,pt_on_pln;
//						for(size_t k=0;k<scan->observed_planes.size();k++)
//						{
//							pt_on_pln=projectPoint2Plane(vec_3d_ref,scan->observed_planes[k]->coefficients,true);
//							vec2plane=pt_on_pln-vec_3d_ref;
//							if(vec2plane.norm()<min)
//							{
//								min=vec2plane.norm();
//								vec_3d_cur=pt_on_pln;
//							}
//						}

						EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
						edge->setVertex(0,pose);
						edge->setMeasurement(vec_3d_cur);
						edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
						edge->setId(i*2);
						optimizer.addEdge(edge);

						pcl::PointXYZRGBA pt1,pt2;
						pt1.x=vec_3d_ref(0);
						pt1.y=vec_3d_ref(1);
						pt1.z=vec_3d_ref(2);
						pt2.x=vec_3d_cur(0);
						pt2.y=vec_3d_cur(1);
						pt2.z=vec_3d_cur(2);
						sprintf(id,"line%d%d",i,j);
						vis->removeShape(id);
						vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);
					}
					else if(points[j]->type==SalientPoint::TYPE::OCCLUDED)
					{ // occluded - find its corresponding occluding point;
						vec_3d_ref=points[j]->xyz+points[j]->proj_ray;
						vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
						query_point[0]=vec_3d_ref(0);
						query_point[1]=vec_3d_ref(1);
						query_point[2]=vec_3d_ref(2);
						kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
						int idx_pln=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
						if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
							continue;
						vec_3d_ref=points[j]->xyz;
						vec_3d_cur=projectPoint2Plane(vec_3d_ref,scan->observed_planes[idx_pln]->coefficients,true);

//						Eigen::Vector4f coeff=scan->planar_regions[idx_pln].getCoefficients();
//						Eigen::Vector3d vec_3d_ref_proj=projectPoint2Plane(vec_3d_ref,coeff);
//						query_point[0]=vec_3d_ref_proj(0);
//						query_point[1]=vec_3d_ref_proj(1);
//						query_point[2]=vec_3d_ref_proj(2);
////						kdtree_occluded->annkSearch(query_point,1,point_index,distance,0);
////						if(distance[0]>0.03)
////							continue;
////							fp<<point_index[0]<<"\t"<<distance[0]<<std::endl;
//
////						int idx=scan->edge_indices[2].indices[point_index[0]];
//						int idx=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
//						vec_3d_cur(0)=scan->point_cloud->at(idx).x;
//						vec_3d_cur(1)=scan->point_cloud->at(idx).y;
//						vec_3d_cur(2)=scan->point_cloud->at(idx).z;

						EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
						edge->setVertex(0,pose);
						edge->setMeasurement(vec_3d_cur);
						edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
						edge->setId(i*2+1);
						optimizer.addEdge(edge);

						pcl::PointXYZRGBA pt1,pt2;
						pt1.x=vec_3d_ref(0);
						pt1.y=vec_3d_ref(1);
						pt1.z=vec_3d_ref(2);
						pt2.x=vec_3d_cur(0);
						pt2.y=vec_3d_cur(1);
						pt2.z=vec_3d_cur(2);
						sprintf(id,"line%d%d",i,j);
						vis->removeShape(id);
						vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,1,0,id);
					}
					else
					{
						continue;
					}
//					// project the occluding point into the cur scan;
//					vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
//					// set the occluding point as the  query point for the kdtree;
////					query_point[0]=vec_3d_ref(0);
////					query_point[1]=vec_3d_ref(1);
////					query_point[2]=vec_3d_ref(2);
////					kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
////					if(distance[0]>0.11)
////						continue;
////					indices.push_back(point_index[0]);
////					dist.push_back(distance[0]);
////					fp<<std::endl<<point_index[0]<<"\t"<<distance[0]<<std::endl;
////
////					// corresponding nearest neighbor in current plane;
////					int idx=scan->edge_indices[1].indices[point_index[0]];
////					int idx_pln=scan->edge_corresponding_plane[1].indices[point_index[0]];
////					if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
////						continue;
////					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
////					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
////					vec_3d_cur(2)=scan->point_cloud->at(idx).z;
//
//					double min=9999999;
//					Eigen::Vector3d vec2plane,pt_on_pln;
//					for(size_t k=0;k<scan->observed_planes.size();k++)
//					{
//						pt_on_pln=projectPoint2Plane(vec_3d_ref,scan->observed_planes[k]->coefficients,true);
//						vec2plane=pt_on_pln-vec_3d_ref;
//						if(vec2plane.norm()<min)
//						{
//							min=vec2plane.norm();
//							vec_3d_cur=pt_on_pln;
//						}
//					}
//
//					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
//					edge->setVertex(0,pose);
//					edge->setMeasurement(vec_3d_cur);
//					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
//					edge->setId(i*2);
//					optimizer.addEdge(edge);
//
//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_ref(0);
//					pt1.y=vec_3d_ref(1);
//					pt1.z=vec_3d_ref(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);

//					// scan->occluding_correspondences.indices[point_index[0]] - corresponding occluded point;
//					// idx_plane - which plane the occluded points is on;
//					if(points[j]->type!=SalientPoint::TYPE::OCCLUDED)
//						continue;
//					int idx_plane=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
//					if(idx_plane>-1)
//					{
//						Eigen::Vector4f coeff=scan->planar_regions[idx_plane].getCoefficients();
//						Eigen::Vector3d vec_3d_ref_proj=projectPoint2Plane(vec_3d_ref,coeff);
//						query_point[0]=vec_3d_ref_proj(0);
//						query_point[1]=vec_3d_ref_proj(1);
//						query_point[2]=vec_3d_ref_proj(2);
////						kdtree_occluded->annkSearch(query_point,1,point_index,distance,0);
////						if(distance[0]>0.03)
////							continue;
//						fp<<point_index[0]<<"\t"<<distance[0]<<std::endl;
//
////						int idx=scan->edge_indices[2].indices[point_index[0]];
//						int idx=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
//						vec_3d_cur(0)=scan->point_cloud->at(idx).x;
//						vec_3d_cur(1)=scan->point_cloud->at(idx).y;
//						vec_3d_cur(2)=scan->point_cloud->at(idx).z;
//
//						EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref_proj);//p_global
//						edge->setVertex(0,pose);
//						edge->setMeasurement(vec_3d_cur);
//						edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
//						edge->setId(i*2+1);
//						optimizer.addEdge(edge);
//
//						pcl::PointXYZRGBA pt1,pt2;
//						pt1.x=vec_3d_ref_proj(0);
//						pt1.y=vec_3d_ref_proj(1);
//						pt1.z=vec_3d_ref_proj(2);
//						pt2.x=vec_3d_cur(0);
//						pt2.y=vec_3d_cur(1);
//						pt2.z=vec_3d_cur(2);
//						sprintf(id,"line%d%d",i,j);
//						vis->removeShape(id);
//						vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,1,0,id);
//
//					}

//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_ref(0);
//					pt1.y=vec_3d_ref(1);
//					pt1.z=vec_3d_ref(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);


//					double min=99999;
//					vec_4d_point.block<3,1>(0,0)=vec_3d_ref;
//					vec_4d_point(3)=1;
//					Plane *matched_plane;
//					for(size_t k=0;k<scan->observed_planes.size();k++)
//					{
//						double tmp=fabs(scan->observed_planes[k]->coefficients.transpose()*vec_4d_point);
//						if(tmp<min)
//						{
//							min=tmp;
////							vec_4d_plane=scan->observed_planes[k]->coefficients;
//							matched_plane=scan->observed_planes[k];
//						}
//					}
//					vec_3d_cur=projectPoint2Plane(vec_3d_ref,matched_plane->coefficients,true);
//
//					vec_3d_pixel=Tcr.intrinsic.project(vec_3d_ref);
//					int col=floor(vec_3d_pixel(0));
//					int row=floor(vec_3d_pixel(1));
//					plane_img.at<cv::Vec3b>(row,col)[0]=0;
//					plane_img.at<cv::Vec3b>(row,col)[1]=255;
//					plane_img.at<cv::Vec3b>(row,col)[2]=0;
//
//					vec_3d_pixel=Tcr.intrinsic.project(vec_3d_cur);
//					col=floor(vec_3d_pixel(0));
//					row=floor(vec_3d_pixel(1));
//					plane_img.at<cv::Vec3b>(row,col)[0]=0;
//					plane_img.at<cv::Vec3b>(row,col)[1]=0;
//					plane_img.at<cv::Vec3b>(row,col)[2]=255;
//
//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_ref(0);
//					pt1.y=vec_3d_ref(1);
//					pt1.z=vec_3d_ref(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);
//
//					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
//					edge->setVertex(0,pose);
//					edge->setMeasurement(vec_3d_cur);
//					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
//					edge->setId(i*2+1);
//					optimizer.addEdge(edge);
				}
				if(debug)
				{
					std::cout<<i<<"th plane done!"<<std::endl;
				}
			}


//			for (size_t i=0;i<scan->edge_label_cloud->height;i++)
//			{
//				for (size_t j=0;j<scan->edge_label_cloud->width;j++)
//				{
//					if(scan->edge_label_cloud->at(j,i).label==2)
//					{ // occluding;
//						plane_img.at<cv::Vec3b>(i,j)[0]=255;
//						plane_img.at<cv::Vec3b>(i,j)[1]=0;
//						plane_img.at<cv::Vec3b>(i,j)[2]=0;
//					}
//					if(scan->edge_label_cloud->at(j,i).label==4)
//					{ // occluded;
//						plane_img.at<cv::Vec3b>(i,j)[0]=255;
//						plane_img.at<cv::Vec3b>(i,j)[1]=255;
//						plane_img.at<cv::Vec3b>(i,j)[2]=255;
//					}
//				}
//			}
//			cv::imshow("img",plane_img);
//			cv::waitKey(0);
			
			if(debug)
			{
				std::cout<<"edge size - "<<optimizer.edges().size()<<std::endl;
			}

			optimizer.initializeOptimization();
			int result=optimizer.optimize ( 30 );
			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
				break;
			}

			Tcr_tmp = pose->estimate();
//			Tcr=Tcr_tmp*Tcr;
			Tcr.leftMultiply(Tcr_tmp);
		} */

		iterations=0;
		while(iterations<max_iter_icp && !converged)
		{
//		    vis->removeAllShapes();
//			fp<<std::endl<<iterations<<"**************************************"<<std::endl;
			if(debug)
				std::cout<<std::endl<<"icp iteration --- "<<iterations<<"**************************************"<<std::endl;
			iterations++;

			chi2_pre=chi2;
			optimizer.clear();

			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setEstimate ( g2o::SE3Quat ( Tcr.R, Tcr.t) );
			pose->setId ( 0 );
			optimizer.addVertex ( pose );

			gettimeofday(&start,NULL);
			indices.clear();
			for(size_t i=0;i<map->planes.size();i++)
			{
				// which planes in map is gonna used to align the current scan;
				if(!inViewFrustum(map,Tcr))
					continue;
//				// points - salient points from map that is to align current observation;
//				points.clear();
//				points.insert(points.end(),map->planes[i]->occluding_points_seq[0].begin(),map->planes[i]->occluding_points_seq[0].end());
//				points.insert(points.end(),map->planes[i]->occluded_points_seq[0].begin(),map->planes[i]->occluded_points_seq[0].end());


				size_t N=map->planes[i]->occluding_points_seq.size();
				occluding.clear();
				occluding.insert(occluding.end(),map->planes[i]->occluding_points_seq[N-1].begin(),
											     map->planes[i]->occluding_points_seq[N-1].end());
				for(size_t j=0;j<occluding.size();j++)
				{
					vec_3d_ref=occluding[j]->xyz;
					// project the occluding point into the cur scan;
					vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
					// set the occluding point as the  query point for the kdtree;
					query_point[0]=vec_3d_ref(0);
					query_point[1]=vec_3d_ref(1);
					query_point[2]=vec_3d_ref(2);
					kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
					if(distance[0]>0.03)
						continue;

					// corresponding nearest neighbor in current plane;
					int idx=scan->edge_indices[1].indices[point_index[0]];
//					int idx_pln=scan->edge_corresponding_plane[1].indices[point_index[0]];
//					if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
//						continue;
					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
					vec_3d_cur(2)=scan->point_cloud->at(idx).z;

					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
					edge->setVertex(0,pose);
					edge->setMeasurement(vec_3d_cur);
					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
					edge->setId(j*3);
					optimizer.addEdge(edge);

//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_ref(0);
//					pt1.y=vec_3d_ref(1);
//					pt1.z=vec_3d_ref(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);
				}

				N=map->planes[i]->occluded_points_seq.size();
				occluded.clear();
				occluded.insert(occluded.end(),map->planes[i]->occluded_points_seq[N-1].begin(),
											   map->planes[i]->occluded_points_seq[N-1].end());
				for(size_t j=0;j<occluded.size();j++)
				{
					vec_3d_ref=occluded[j]->xyz+occluded[j]->proj_ray;
					vec_3d_ref=Tcr.R*vec_3d_ref+Tcr.t;
					query_point[0]=vec_3d_ref(0);
					query_point[1]=vec_3d_ref(1);
					query_point[2]=vec_3d_ref(2);
					kdtree_occluding->annkSearch(query_point,1,point_index,distance,0);
					if(distance[0]>0.03)
						continue;

					// corresponding nearest neighbor in current plane;
					int idx=scan->edge_indices[1].indices[point_index[0]];
//					int idx_pln=scan->edge_corresponding_plane[1].indices[point_index[0]];
//					if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
//						continue;
					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
					vec_3d_cur(2)=scan->point_cloud->at(idx).z;

					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
					edge->setVertex(0,pose);
					edge->setMeasurement(vec_3d_cur);
					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
					edge->setId(j*3+1);
					optimizer.addEdge(edge);

//					int idx_pln=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
//					if(idx_pln==-1 || !scan->observed_planes[idx_pln]->similar(map->planes[i],0.6,0.5))
//						continue;
//					vec_3d_ref=occluded[j]->xyz;

//					int idx=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
//					vec_3d_cur(0)=scan->point_cloud->at(idx).x;
//					vec_3d_cur(1)=scan->point_cloud->at(idx).y;
//					vec_3d_cur(2)=scan->point_cloud->at(idx).z;
//
//					EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref);//p_global
//					edge->setVertex(0,pose);
//					edge->setMeasurement(vec_3d_cur);
//					edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
//					edge->setId(i*2+1);
//					optimizer.addEdge(edge);

//					pcl::PointXYZRGBA pt1,pt2;
//					pt1.x=vec_3d_ref(0);
//					pt1.y=vec_3d_ref(1);
//					pt1.z=vec_3d_ref(2);
//					pt2.x=vec_3d_cur(0);
//					pt2.y=vec_3d_cur(1);
//					pt2.z=vec_3d_cur(2);
//					sprintf(id,"line%d%d",i,j);
//					vis->removeShape(id);
//					vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,1,1,1,id);


					// corresponding occluded point in current scan is on plane [idx_plane];
					if(scan->occluding_correspondences.indices[point_index[0]]==-1)
						continue;
					int idx_plane=scan->edge_corresponding_plane[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
					if(idx_plane>-1)
					{
						Eigen::Vector4f coeff=scan->planar_regions[idx_plane].getCoefficients();
						Eigen::Vector3d vec_3d_ref_proj=projectPoint2Plane(vec_3d_ref,coeff);
						query_point[0]=vec_3d_ref_proj(0);
						query_point[1]=vec_3d_ref_proj(1);
						query_point[2]=vec_3d_ref_proj(2);
						kdtree_occluded->annkSearch(query_point,1,point_index,distance,0);
						if(distance[0]>0.03)
							continue;
//						fp<<point_index[0]<<"\t"<<distance[0]<<std::endl;

						int idx=scan->edge_indices[2].indices[point_index[0]];
//						int idx=scan->edge_indices[2].indices[scan->occluding_correspondences.indices[point_index[0]]];
						vec_3d_cur(0)=scan->point_cloud->at(idx).x;
						vec_3d_cur(1)=scan->point_cloud->at(idx).y;
						vec_3d_cur(2)=scan->point_cloud->at(idx).z;

						EdgeSE3pra *edge=new EdgeSE3pra(vec_3d_ref_proj);//p_global
						edge->setVertex(0,pose);
						edge->setMeasurement(vec_3d_cur);
						edge->setInformation(Eigen::Matrix<double,3,3>::Identity());
						edge->setId(j*3+2);
						optimizer.addEdge(edge);

//						pcl::PointXYZRGBA pt1,pt2;
//						pt1.x=vec_3d_ref_proj(0);
//						pt1.y=vec_3d_ref_proj(1);
//						pt1.z=vec_3d_ref_proj(2);
//						pt2.x=vec_3d_cur(0);
//						pt2.y=vec_3d_cur(1);
//						pt2.z=vec_3d_cur(2);
//						sprintf(id,"line%d%d",i,j);
//						vis->removeShape(id);
//						vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,1,0,id);

					}
				}

				if(debug)
				{
					std::cout<<"pose estimating: "<<i<<"th plane in the map!"<<std::endl;
					std::cout<<"real_contour: "<<map->planes[i]->real_contour.size()<<std::endl;
					std::cout<<"occluding_points_seq[0]: "<<occluding.size()<<std::endl;
					std::cout<<"occluded_points_seq[0]: "<<occluded.size()<<std::endl;
				}

			}
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;

			if(debug)
			{
				std::cout<<"time adding edges:"<<timeused<<std::endl;
				std::cout<<"edge size - "<<optimizer.edges().size()<<std::endl;
			}

			gettimeofday(&start,NULL);
			optimizer.initializeOptimization();
			int result=optimizer.optimize ( max_iter_g2o );
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			if(debug)
				std::cout<<"time LM optimizing:"<<timeused<<std::endl;

			chi2=optimizer.activeChi2();
			if(chi2>chi2_pre)
			{
				break;
			}

			Tcr_tmp = pose->estimate();
			Tcr.leftMultiply(Tcr_tmp);

//				cloud_ref->resize(scan_ref->point_cloud->size());
//				for(size_t i=0;i<scan_ref->point_cloud->size();i++)
//				{
//					cloud_ref->at(i).x=scan_ref->point_cloud->at(i).x;
//					cloud_ref->at(i).y=scan_ref->point_cloud->at(i).y;
//					cloud_ref->at(i).z=scan_ref->point_cloud->at(i).z;
//					cloud_ref->at(i).r=255;
//					cloud_ref->at(i).g=0;
//					cloud_ref->at(i).b=0;
//				}
//				cloud_cur->resize(scan->point_cloud->size());
//				for(size_t i=0;i<scan->point_cloud->size();i++)
//				{
//					cloud_cur->at(i).x=scan->point_cloud->at(i).x;
//					cloud_cur->at(i).y=scan->point_cloud->at(i).y;
//					cloud_cur->at(i).z=scan->point_cloud->at(i).z;
//					cloud_cur->at(i).r=0;
//					cloud_cur->at(i).g=0;
//					cloud_cur->at(i).b=255;
//				}
//				// show point clouds before transformation;
//				vis->removeAllPointClouds();
//				if (!vis->updatePointCloud (cloud_cur, "cur"))
//					vis->addPointCloud (cloud_cur,"cur");
//				if (!vis->updatePointCloud (cloud_ref, "ref"))
//					vis->addPointCloud (cloud_ref,"ref");
//				vis->spin();
//				// show point clouds after transformation;
//				vis->removeAllPointClouds();
//				Eigen::Matrix4f trans=Tcr.getMatrix4f();
//				transformPointCloud(*cloud_ref,*cloud_ref,trans);
//				if (!vis->updatePointCloud (cloud_cur, "cur"))
//					vis->addPointCloud (cloud_cur,"cur");
//				if (!vis->updatePointCloud (cloud_ref, "ref"))
//					vis->addPointCloud (cloud_ref,"ref");
//				vis->spin();
		}



		// planes in map are described in global frame;
		// so Tcg=Tcr;
		scan->Tcg->R=Tcr.R;
		scan->Tcg->t=Tcr.t;

		delete point_index;
		delete distance;
		delete kdtree_occluding;
		delete kdtree_occluded;
		annDeallocPt(query_point);
		annDeallocPts(occluding_cur);
		annDeallocPts(occluded_cur);
		fp.close();
	}

	bool PoseEstimation::inViewFrustum(Map *map, Transform pose)
	{
		return true;
	}
	
}

