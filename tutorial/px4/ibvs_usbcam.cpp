/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

//! \example tutorial-franka-coppeliasim-ibvs-apriltag.cpp

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

// Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

#include "utils.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

int _len_traj;
Eigen::MatrixXd _EE_corner_px_pos;       
std::vector< vpImagePoint > ee_corners_1,ee_corners_2,ee_corners_3,ee_corners_4;
bool ee_flag;

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      // Add the point only if distance with the previous > 1 pixel
      if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
      {
        traj_vip[i].push_back( vip[i] );
      }
    }
    else
    {
      traj_vip[i].push_back( vip[i] );
    }
  }
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    for ( size_t j = 1; j < traj_vip[i].size(); j++ )
    {
      //vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
    }
  }
}

void ee_pointsCallback(const std_msgs::Float64MultiArray msg){
	_len_traj = msg.data[0];
  _EE_corner_px_pos.resize(2,_len_traj/2);
	_EE_corner_px_pos<< msg.data[1],msg.data[3],msg.data[5],msg.data[7],msg.data[2],msg.data[4],msg.data[6],msg.data[8];
  // cout<<"EE corners: \n"<<_EE_corner_px_pos<<endl;
  ee_flag = true;
}

int
main( int argc, char **argv )
{
  double opt_tagSize             = 0.08; // era 0.08
  bool display_tag               = true;
  int opt_quad_decimate          = 2;
  bool opt_verbose               = false;
  bool opt_plot                  = false;
  bool opt_adaptive_gain         = false;
  bool opt_task_sequencing       = false;
  double convergence_threshold   = 0.0005;
  bool opt_coppeliasim_sync_mode = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--tag_size" && i + 1 < argc )
    {
      opt_tagSize = std::stod( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--plot" )
    {
      opt_plot = true;
    }
    else if ( std::string( argv[i] ) == "--adaptive_gain" )
    {
      opt_adaptive_gain = true;
    }
    else if ( std::string( argv[i] ) == "--task_sequencing" )
    {
      opt_task_sequencing = true;
    }
    else if ( std::string( argv[i] ) == "--quad_decimate" && i + 1 < argc )
    {
      opt_quad_decimate = std::stoi( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--no-convergence-threshold" )
    {
      convergence_threshold = 0.;
    }
    else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] "
                << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] "
                << "[--adaptive_gain] "
                << "[--plot] "
                << "[--task_sequencing] "
                << "[--no-convergence-threshold] "
                << "[--enable-coppeliasim-sync-mode] "
                << "[--verbose] [-v] "
                << "[--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }
  _EE_corner_px_pos.resize(2,4);
  bool print = false;
  try
  {
    //------------------------------------------------------------------------//
    //------------------------------------------------------------------------//
    // ROS node
    ros::init( argc, argv, "visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::NodeHandle _n;
    ros::Publisher point_pub = _n.advertise<geometry_msgs::Pose>("/drone/camera/tag_pose", 0);
    ros::Publisher flag_pub = _n.advertise<std_msgs::Bool>("/drone/camera/tag_flag", 0);
 		ros::Publisher _vs_pub = _n.advertise< std_msgs::Float64MultiArray >("/drone/tag_pos_px_corner", 0);

    ros::Subscriber ee_sub = _n.subscribe("/kuka/ee_pos_corner", 1000, ee_pointsCallback);
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    vpImage< unsigned char > I;
    vpROSGrabber g;
    g.setImageTopic( "/iris/usb_cam/image_raw" );
    g.setCameraInfoTopic( "/iris/usb_cam/camera_info" );
    g.open( argc, argv );

    g.acquire( I );

    std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
    vpCameraParameters cam;

    g.getCameraInfo( cam );
    std::cout << cam << std::endl;
    vpDisplayOpenCV dc( I, 10, 10, "Color image" );

    vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector( tagFamily );
    detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
    detector.setDisplayTag( display_tag );
    detector.setAprilTagQuadDecimate( opt_quad_decimate );

    // Servo
    vpHomogeneousMatrix cMo, oMo;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo( vpTranslationVector( 0.0, -0.00, opt_tagSize * 4 - 0.055 ), //era 0.0, -0.06, opt_tagSize * 2 - 0.055
    // vpHomogeneousMatrix cdMo( vpTranslationVector( 0.0, -0.01175, opt_tagSize * 4 - 0.003 ),
                              vpRotationMatrix( { 1, 0, 0, 0, 1, 0, 0, 0, 1 } ) );

    // Create visual features
    std::vector< vpFeaturePoint > p( 4 ), pd( 4 ); // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector< vpPoint > point( 4 );
    point[0].setWorldCoordinates( -opt_tagSize / 2., opt_tagSize / 2., 0 );
    point[1].setWorldCoordinates(  opt_tagSize / 2., opt_tagSize / 2., 0 );
    point[2].setWorldCoordinates( opt_tagSize / 2.,  -opt_tagSize / 2., 0 );
    point[3].setWorldCoordinates(  -opt_tagSize / 2.,  -opt_tagSize / 2., 0 );

    vpServo task;
    // Add the 4 visual feature points
    for ( size_t i = 0; i < p.size(); i++ )
    {
      task.addFeature( p[i], pd[i] );
    }
    task.setServo( vpServo::EYEINHAND_CAMERA );
    task.setInteractionMatrixType( vpServo::CURRENT );

    if ( opt_adaptive_gain )
    {
      std::cout << "Enable adaptive gain" << std::endl;
      vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
      task.setLambda( lambda );
    }
    else
    {
      task.setLambda( 1.2 );
    }

    vpPlot *plotter = nullptr;

    if ( opt_plot )
    {
      plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
                            "Real time curves plotter" );
      plotter->setTitle( 0, "Visual features error" );
      plotter->setTitle( 1, "Camera velocities" );
      plotter->initGraph( 0, 8 );
      plotter->initGraph( 1, 6 );
      plotter->setLegend( 0, 0, "error_feat_p1_x" );
      plotter->setLegend( 0, 1, "error_feat_p1_y" );
      plotter->setLegend( 0, 2, "error_feat_p2_x" );
      plotter->setLegend( 0, 3, "error_feat_p2_y" );
      plotter->setLegend( 0, 4, "error_feat_p3_x" );
      plotter->setLegend( 0, 5, "error_feat_p3_y" );
      plotter->setLegend( 0, 6, "error_feat_p4_x" );
      plotter->setLegend( 0, 7, "error_feat_p4_y" );
      plotter->setLegend( 1, 0, "vc_x" );
      plotter->setLegend( 1, 1, "vc_y" );
      plotter->setLegend( 1, 2, "vc_z" );
      plotter->setLegend( 1, 3, "wc_x" );
      plotter->setLegend( 1, 4, "wc_y" );
      plotter->setLegend( 1, 5, "wc_z" );
    }

    bool final_quit                           = false;
    bool has_converged                        = false;
    bool send_velocities                      = false;
    bool servo_started                        = false;
    std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory
    double sim_time            = 0.0;
    double sim_time_prev       = sim_time;
    double sim_time_init_servo = sim_time;
    double sim_time_img        = sim_time;
    std_msgs::Bool found;
    found.data = false;

    if ( 0 )
    {
      // Instead of setting eMc from /coppeliasim/franka/eMc topic, we can set its value to introduce noise for example
      vpHomogeneousMatrix eMc;
      eMc.buildFrom( 0.05, -0.05, 0, 0, 0, M_PI_4 );
    }

    while ( !final_quit )
    {
      sim_time += 1/1000;

      g.acquire( I, sim_time_img );
      vpDisplay::display( I );

      std::vector< vpHomogeneousMatrix > cMo_vec;
      detector.detect( I, opt_tagSize, cam, cMo_vec );
//      cMo = cMo_vec[0];
//      std::cout<<detector.getPose(0, opt_tagSize, cam, cMo);
//      std::cout<<"\n"<<cMo<<"\n"<<std::endl;

      {
        std::stringstream ss;
        // ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
        //    << ", right click to quit.";
        ss << "Press right click to quit.";
        vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
      }

      vpColVector v_c( 6 );
      Eigen::MatrixXd T_detected;
      Eigen::Vector4d quat_detected;
      Eigen::Vector3d rpy_detected;
      T_detected.resize(4,4);
      geometry_msgs::Pose pose;

      if ( cMo_vec.size() == 1 )       // Only one tag is detected // if ( cMo_vec.size() < 3 )           // Only one tag is detected or 2 at max
      {
        found.data = true;
        cMo = cMo_vec[0];
        // std::cout<<"Pose detected: \n["<<cMo<<"]\n"<<std::endl;
        T_detected <<   cMo[0][0], cMo[0][1],cMo[0][2],cMo[0][3],
                        cMo[1][0], cMo[1][1],cMo[1][2],cMo[1][3],
                        cMo[2][0], cMo[2][1],cMo[2][2],cMo[2][3],
                        cMo[3][0], cMo[3][1],cMo[3][2],cMo[3][3];
        // std::cout<<"Pose converted: \n["<<T_detected<<"]\n"<<std::endl;
        quat_detected = utilities::rot2quat( T_detected.block<3,3>(0,0) );
        rpy_detected = utilities::MatToRpy(T_detected.block<3,3>(0,0));

        pose.position.x = T_detected(0,3);
        pose.position.y = T_detected(1,3);
        pose.position.z = T_detected(2,3);
        pose.orientation.x = quat_detected(1);
        pose.orientation.y = quat_detected(2);
        pose.orientation.z = quat_detected(3);
        pose.orientation.w = quat_detected(0);
        // pose.orientation.x = rpy_detected(0);
        // pose.orientation.y = rpy_detected(1);
        // pose.orientation.z = rpy_detected(2);
        // pose.orientation.w = 0.0;
        point_pub.publish(pose);
        
        static bool first_time = true;
        if ( first_time )
        {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector< vpHomogeneousMatrix > v_oMo( 2 ), v_cdMc( 2 );
          v_oMo[1].buildFrom( 0, 0, 0, 0, 0, M_PI );
          for ( size_t i = 0; i < 2; i++ )
          {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
          // if ( std::fabs( v_cdMc[0].getThetaUVector().getTheta() ) <
          //      std::fabs( v_cdMc[1].getThetaUVector().getTheta() ) )
          // {
            oMo = v_oMo[0];
          // }
          // else
          // {
          //   std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
          //   oMo = v_oMo[1]; // Introduce PI rotation
          // }

          // Compute the desired position of the features from the desired pose
          std::cout << "Initializing desired pose" << std::endl;
          for ( size_t i = 0; i < point.size(); i++ )
          {
            vpColVector cP, p_;
            point[i].changeFrame( cdMo * oMo, cP );
            point[i].projection( cP, p_ );

            pd[i].set_x( p_[0] );
            pd[i].set_y( p_[1] );
            pd[i].set_Z( cP[2] );
          }
        } // end first_time

        // Get tag corners
        std::vector< vpImagePoint > corners = detector.getPolygon( 0 );
        // for (int i=0; i<corners.size();i++){
        //   cout << "vector[ "<<i<<" ]: " <<corners[i] << endl;
        // }
        std_msgs::Float64MultiArray corners_px;
        corners_px.data.resize(9);
        corners_px.data[0] = 8;
        corners_px.data[1] = corners[0].get_i();
        corners_px.data[2] = corners[0].get_j();
        corners_px.data[3] = corners[1].get_i();
        corners_px.data[4] = corners[1].get_j();
        corners_px.data[5] = corners[2].get_i();
        corners_px.data[6] = corners[2].get_j();
        corners_px.data[7] = corners[3].get_i();
        corners_px.data[8] = corners[3].get_j();
		    _vs_pub.publish(corners_px);

        // Update visual features
        for ( size_t i = 0; i < corners.size(); i++ )
        {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create( p[i], cam, corners[i] );
          // Set the feature Z coordinate from the pose
          vpColVector cP;
          point[i].changeFrame( cMo, cP );

          p[i].set_Z( cP[2] );
        }

        // Display the current and desired feature points in the image display
        vpServoDisplay::display( task, cam, I );
        // vpDisplay::displayPoint(I, 537.669, 318.506, vpColor::green, 4);
        // vpDisplay::displayPoint(I, 537.669, 406.702, vpColor::green, 4);
        // vpDisplay::displayPoint(I, 449.474, 406.702, vpColor::green, 4);
        // vpDisplay::displayPoint(I, 449.474, 318.506, vpColor::green, 4);

        for ( size_t i = 0; i < corners.size(); i++ )
        {
          std::stringstream ss;
          ss << i;
          // Display current point indexes
          vpDisplay::displayText( I, corners[i] + vpImagePoint( 15, 15 ), ss.str(), vpColor::red );
          // Display desired point indexes
          vpImagePoint ip;
          vpMeterPixelConversion::convertPoint( cam, pd[i].get_x(), pd[i].get_y(), ip );
          vpDisplay::displayText( I, ip + vpImagePoint( 15, 15 ), ss.str(), vpColor::red );
        }
        if ( first_time )
        {
          traj_corners = new std::vector< vpImagePoint >[corners.size()];
        }
        // Display the trajectory of the points used as features
        display_point_trajectory( I, corners, traj_corners );
                    
        if(ee_flag){
          print = true;
        }
        if (_EE_corner_px_pos(0,0)>1 && _EE_corner_px_pos(0,1)>1 && _EE_corner_px_pos(0,2)>1 && _EE_corner_px_pos(0,3)>1 ) {
          ee_corners_1.push_back(vpImagePoint(_EE_corner_px_pos(1,0), _EE_corner_px_pos(0,0)));
          ee_corners_2.push_back(vpImagePoint(_EE_corner_px_pos(1,1), _EE_corner_px_pos(0,1)));
          ee_corners_3.push_back(vpImagePoint(_EE_corner_px_pos(1,2), _EE_corner_px_pos(0,2)));
          ee_corners_4.push_back(vpImagePoint(_EE_corner_px_pos(1,3), _EE_corner_px_pos(0,3)));
          
          vpDisplay::displayPoint(I, _EE_corner_px_pos(1,0), _EE_corner_px_pos(0,0), vpColor::red, 2);
          vpDisplay::displayPoint(I, _EE_corner_px_pos(1,1), _EE_corner_px_pos(0,1), vpColor::red, 2);
          vpDisplay::displayPoint(I, _EE_corner_px_pos(1,2), _EE_corner_px_pos(0,2), vpColor::red, 2);
          vpDisplay::displayPoint(I, _EE_corner_px_pos(1,3), _EE_corner_px_pos(0,3), vpColor::red, 2);
          
          vpDisplay::displayText( I, vpImagePoint(_EE_corner_px_pos(1,0), _EE_corner_px_pos(0,0)) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::darkBlue );
          vpDisplay::displayText( I, vpImagePoint(_EE_corner_px_pos(1,1), _EE_corner_px_pos(0,1)) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::darkBlue );
          vpDisplay::displayText( I, vpImagePoint(_EE_corner_px_pos(1,2), _EE_corner_px_pos(0,2)) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::darkBlue );
          vpDisplay::displayText( I, vpImagePoint(_EE_corner_px_pos(1,3), _EE_corner_px_pos(0,3)) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::darkBlue ); 

          // vpDisplay::displayPoint(I, vpImagePoint(588.938, 257.192), vpColor::darkGreen, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(588.938, 546.738), vpColor::darkGreen, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(299.392, 546.738), vpColor::darkGreen, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(299.392, 257.192), vpColor::darkGreen, 2);
          
          // vpDisplay::displayText( I, vpImagePoint(588.938, 257.192) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::darkGreen );
          // vpDisplay::displayText( I, vpImagePoint(588.938, 546.738) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::darkGreen );
          // vpDisplay::displayText( I, vpImagePoint(299.392, 546.738) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::darkGreen );
          // vpDisplay::displayText( I, vpImagePoint(299.392, 257.192) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::darkGreen );
           
          // vpDisplay::displayPoint(I, vpImagePoint(511.963, 342.771 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(511.963, 377.426 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(477.308, 377.426 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(477.308, 342.771 ), vpColor::green, 2);
          
          // vpDisplay::displayText( I, vpImagePoint(511.963, 342.771 ) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(511.963, 377.426 ) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(477.308, 377.426 ) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(477.308, 342.771 ) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::green ); 

          // vpDisplay::displayPoint(I, vpImagePoint(504.948, 347.842 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(504.945, 383.187 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(469.618, 383.171 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(469.621, 347.826 ), vpColor::green, 2);
          
          // vpDisplay::displayText( I, vpImagePoint(504.948, 347.842 ) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(504.945, 383.187 ) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(469.618, 383.171 ) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(469.621, 347.826 ) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::green );
   
          // vpDisplay::displayPoint(I, vpImagePoint(516.335, 342.185 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(517.071, 377.025 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(482.809, 378.751 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(481.917, 343.849 ), vpColor::green, 2);
          
          // vpDisplay::displayText( I, vpImagePoint(516.335, 342.185 ) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(517.071, 377.025 ) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(482.809, 378.751 ) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(481.917, 343.849 ) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::green ); 

          // vpDisplay::displayPoint(I, vpImagePoint(485.216, 356.509 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(488.264, 391.076 ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(453.48, 395.015  ), vpColor::green, 2);
          // vpDisplay::displayPoint(I, vpImagePoint(450.259, 360.552 ), vpColor::green, 2);
          
          // vpDisplay::displayText( I, vpImagePoint(485.216, 356.509 ) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(488.264, 391.076 ) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(453.48, 395.015  ) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::green );
          // vpDisplay::displayText( I, vpImagePoint(450.259, 360.552 ) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::green );

          vpDisplay::displayPoint(I, vpImagePoint(491.181, 353.592 ), vpColor::green, 2);
          vpDisplay::displayPoint(I, vpImagePoint(491.181, 384.864 ), vpColor::green, 2);
          vpDisplay::displayPoint(I, vpImagePoint(459.91, 384.864 ), vpColor::green, 2);
          vpDisplay::displayPoint(I, vpImagePoint(459.91, 353.592 ), vpColor::green, 2);
          
          vpDisplay::displayText( I, vpImagePoint(491.181, 353.592 ) + vpImagePoint( 10, 10 ), stringstream("0").str(), vpColor::green );
          vpDisplay::displayText( I, vpImagePoint(491.181, 384.864 ) + vpImagePoint( 10, 10 ), stringstream("1").str(), vpColor::green );
          vpDisplay::displayText( I, vpImagePoint(459.91, 384.864 ) + vpImagePoint( 10, 10 ), stringstream("2").str(), vpColor::green );
          vpDisplay::displayText( I, vpImagePoint(459.91, 353.592 ) + vpImagePoint( 10, 10 ), stringstream("3").str(), vpColor::green );

        }  
        if (print ==true){
          vpDisplay::displayLine(I,ee_corners_1,false,vpColor::darkBlue,2);
          vpDisplay::displayLine(I,ee_corners_2,false,vpColor::darkBlue,2);
          vpDisplay::displayLine(I,ee_corners_3,false,vpColor::darkBlue,2);
          vpDisplay::displayLine(I,ee_corners_4,false,vpColor::darkBlue,2);
        }
        


        if ( opt_plot )
        {
          plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
          plotter->plot( 1, static_cast< double >( sim_time ), v_c );
        }

        if ( opt_verbose )
        {
          std::cout << "v_c: " << v_c.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        std::stringstream ss;
        ss << "||error||: " << error;
        // vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );

        if ( opt_verbose )
          std::cout << ss.str() << std::endl;

        if ( !has_converged && error < convergence_threshold )
        {
          has_converged = true;
          std::cout << "Servo task has converged"
                    << "\n";
          // vpDisplay::displayText( I, 100, 20, "Servo task has converged", vpColor::red );
        }

        if ( first_time )
        {
          first_time = false;
        }
      } // end if (cMo_vec.size() == 1)
      else
      {
        v_c = 0; // Stop the robot
        found.data = false;
      }
      flag_pb.publish(found);
      if ( !send_velocities )
      {
        v_c = 0; // Stop the robot
      }

      std::stringstream ss;
      ss << "Loop time [s]: " << std::round( ( sim_time - sim_time_prev ) * 1000. ) / 1000.;
      ss << " Simulation time [s]: " << sim_time;
      sim_time_prev = sim_time;
      // vpDisplay::displayText( I, 40, 20, ss.str(), vpColor::red );

      vpMouseButton::vpMouseButtonType button;
      if ( vpDisplay::getClick( I, button, false ) )
      {
        switch ( button )
        {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          v_c        = 0;
          break;

        default:
          break;
        }
      }

      vpDisplay::flush( I );
    }                                // end while

    if ( opt_plot && plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }

    if ( !final_quit )
    {
      while ( !final_quit )
      {
        g.acquire( I );
        vpDisplay::display( I );

        // vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
        // vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

        if ( vpDisplay::getClick( I, false ) )
        {
          final_quit = true;
        }

        vpDisplay::flush( I );
      }
    }
    if ( traj_corners )
    {
      delete[] traj_corners;
    }
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
