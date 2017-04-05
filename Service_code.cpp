/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Service_code.cpp
 * Author: Peter Atrazhev
 *
 * Created on April 5, 2017, 4:56 PM
 */


#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#include <stdlib.h>
#include <cmath> // std::fabs
#include <limits> // numeric_limits
#include <list>
#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394))


#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/blob/vpDot.h>
#include <visp3/vision/vpPose.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGTK.h>
#include "ros/ros.h"
#include 'std_msg/String.h"

#include <sstream>
using namespace std;

/*
 * 
 */

// this is the WAM Jacobean callback function 
void wamToolJacobeanCallback( const wam_msg::MatrixMN:ConstPtr& jacobeanMessage)
{
// this was taken from fuego
	for (int j = 0; j < 7; j++)
	{
	 for (int i = 0; i < 6; i++)
		{
		 fJe(i,j)=jacobeanMessage->data[i+6*j];

		} 
	}	

}

bool joint_move_callback


int main(int argc, char** argv) {

    
    // this is how to call the jacobean subscriber 
    ros::Subscriber jacobean_sub = nh.subscribe("zeus/wam/jacobian",1,wamToolJacobeanCallBack);

    
    // this is how to calculate the intergration based on the two timestamps 
    
    // this is how to call the joint_move service 
    
    // send the joint vector to the ros service 
    
    
    return 0;
}

