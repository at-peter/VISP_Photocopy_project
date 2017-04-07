#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include "ros/ros.h"


vpMatrix fJe(6,7) ;

void wamToolJacobeanCallback(const wam_msg::MatrixMN::ConstPtr& jacobeanMessage)
{
    for (int j = 0; j < 7; j++)
	{
	 for (int i = 0; i < 6; i++)
		{
		 fJe(i,j)=jacobeanMessage->data[i+6*j];

		} 
	}	
}

vpColvector position_integration(vpColVector prev_vel,vpColVector current_vel, float t_delta )
{   
    vpColVector position(7);
    for (i=0; i<7,i++)
    {
        position[i]=((current_vel[i] - prev_vel[i])*t_delta)/2;
    }
    
    return position;
}
int main()
{
    // ros::init(not sure if we need this);
    ros::NodeHandle nh;
    ros::ServiceClient Joint_move_client = nh.serviceClient<wam_node::wam_srvs>("JointMove");
    
    wam_node::wam_srvs mv_srv
            
  try {
   ros::Subscriber jacobean_sub = nh.subscribe("zeus/wam/jacobian",1,wamToolJacobeanCallBack);
   vpServo task ;
   vpFeaturePoint p(100,100);  //current point
   vpFeaturePoint pd(200,200); //desired point
    //------------------------------------------------------------------
    vpTRACE("define the task") ;
    vpTRACE("\t we want an eye-in-hand control law") ;
    vpTRACE("\t robot is controlled in the camera frame") ;
   task.setServo(vpServo::EYETOHAND_L_cVf_fJe) ;
   task.setInteractionMatrixType(vpServo::CURRENT) ;
   task.addFeature(p,pd) ;
   vpTRACE("Display task information " ) ;
    task.print() ;
   
   
     double convergence_threshold = 0.00; //025 ;
      double error =1 ;
    unsigned int iter=0 ;
    vpTRACE("\t loop") ;
    
    vpHomogeneousMatrix cMf ;//camera's position wrt robot base reference frame 4*4
    cMf[0][3] = -0.05 ;
    task.set_cVf(cMf);
    double lambda_av =0.2;
    task.setLambda(lambda);
    
    
    vpImage<vpRGBa> Ic ;
    int it = 0 ;
   
    double alpha = 1 ; //1 ;
    double beta =3 ; //3 ;
    std::cout << "alpha 0.7" << std::endl;
    std::cin >> alpha ;
    std::cout << "beta 5" << std::endl;
    std::cin >> beta ;
    std::list<vpImagePoint> Lcog ;
    vpImagePoint ip;
    
    double t1=0;
    double t2=0;
    // define the service outside the while loop 
    vpColVector dotq_prev (7); // this is to hold the previous velocities to integrate
    
      while(error > convergence_threshold)
    {
       t1 = vpTime::measureTimeMs();
      
      std::cout << "---------------------------------------------" << iter++ <<std::endl ;
      vpColVector dotq(7);
      vpColVector integrated_pos(7); // this is to store the position
      //robot.get_eJe(eJe) ;
      //get fJe from topic jacobian/////////////////////////////////////////////////////////
      //vpMatrix fJe(6,7);
      //define fJe;
      //fJe[0][0]=0;
      task.set_fJe(fJe) ; //in while loop, always update fJe
      //we also need to update the current point position in the image.////use it in the ros topic subscriber.
      
      
      
      // Compute the adaptative gain (speed up the convergence)
      double gain ;
      if (iter>2)
      {
        if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
          gain = lambda_av ;
        else
        {
          gain = alpha * exp (-beta * ( task.getError() ).sumSquare() ) +  lambda_av;
        }
      }
      else gain = lambda_av ;
//       if (SAVE==1)
//         gain = gain/5 ;
      vpTRACE("%f %f %f %f  %f",alpha, beta, lambda_av, ( task.getError() ).sumSquare(),  gain) ;
      task.setLambda(gain) ;
      
      dotq = task.computeControlLaw() ;
      //
      // robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ; //set the joint velocity to the robot
      //using ros service to set the joint velocity to the robot.///////////////////////////////////////////////////////////////
      
      error = ( task.getError() ).sumSquare() ;
      std::cout << "|| s - s* || = "<< error<<std::endl ;
      
      if (error>7)
        {
          vpTRACE("Error detected while tracking visual features") ;
         /////////////////////
	  //using ros service to set the robot to stop.////////////////////////////////////////////////////////////////////////
	  
          exit(1) ;
        }
        
           t2 = vpTime::measureTimeMs();
           // integration function goes here 
           delta_t= t2-t1;
           integrated_pos = position_integration(dotq_prev,dotq,delta_t);
           
           //then we update the position using the rosservice
           mv_srv.response.a = integrated_pos(0);
           mv_srv.response.b = integrated_pos(1);
           mv_srv.response.c = integrated_pos(2);
           mv_srv.response.e = integrated_pos(3);
           mv_srv.response.f = integrated_pos(4);
           mv_srv.response.g = integrated_pos(5);
           mv_srv.response.h = integrated_pos(6);
           
           // call the service 
           if (client.call(mv_srv))
           {
 
           }
           else {
               ROS_ERROR("Failed to call service Joint Move")
           }
           
           dotq_prev=dotq;// save the velocity information 
           
    }
    
    //robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;///////////////////////////////////////////////////////using ros to stop the robot motion
    
    task.kill();
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
   mv_srv.response.a = integrated_pos(0);
           mv_srv.response.b = 0;
           mv_srv.response.c = 0;
           mv_srv.response.e = 0;
           mv_srv.response.f = 0;
           mv_srv.response.g = 0;
           mv_srv.response.h = 0;
           
           // call the service 
           if (client.call(mv_srv))
           {
 
           }
           else {
               ROS_ERROR("Failed to call service Joint Move when there was an exception")
           } 
  }
}
