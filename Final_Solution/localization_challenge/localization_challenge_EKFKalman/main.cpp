#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"
#include <iostream>
#include <eigen3/Eigen/Dense>



/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
    // TODO: Write your procedures to set the current robot position estimate here
    
   estimatePosn.x = eigRobotState(0,0);
   estimatePosn.y = eigRobotState(1,0);
   estimatePosn.theta = eigRobotState(2,0);
   


}

/** 
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // TODO: Write your motion update procedures here
    double angle = -eigRobotState(2,0);
    eigRobotState(0,0) = eigRobotState(0,0) + cos(angle-delta.theta)*delta.x + sin(angle-delta.theta)*delta.y;
    eigRobotState(1,0) = eigRobotState(1,0) - sin(angle-delta.theta)*delta.x + cos(angle-delta.theta)*delta.y;
    eigRobotState(2,0) = eigRobotState(2,0) + delta.theta;

    // wrap angle around
    // if(eigRobotState(2,0)>M_PI)
    {
    //     eigRobotState(2,0) = eigRobotState(2,0) - 2*M_PI;
    // }
    // else if(eigRobotState(2,0)<-M_PI)
    {
    //     eigRobotState(2,0) = eigRobotState(2,0) + 2*M_PI;
    // }
    
    Eigen::MatrixXd  G = Eigen::MatrixXd::Zero(3,3);

    G(0,0) = 1;
    G(0,2) = -sin(angle-delta.theta)*delta.x + cos(angle-delta.theta)*delta.y;
    
    G(1,1) = 1;
    G(1,2) = -cos(angle-delta.theta)*delta.x -sin(angle-delta.theta)*delta.y;
    
    G(2,2) = 1;

    Eigen::MatrixXd  Fx = Eigen::MatrixXd::Zero(3,3*NUM_LANDMARKS + 3);

    Fx(0,0) = 1;
    Fx(1,1) = 1;
    Fx(2,2) = 1;

    Eigen::MatrixXd  temp_F = Eigen::MatrixXd::Zero(3,3);

    temp_F(0,0) = cos(angle-delta.theta);
    temp_F(0,1) = sin(angle-delta.theta);
    temp_F(0,2) = sin(angle-delta.theta)*delta.x - cos(angle-delta.theta)*delta.y;

    temp_F(1,0) = -sin(angle-delta.theta);
    temp_F(1,1) = cos(angle-delta.theta);
    temp_F(1,2) = cos(angle-delta.theta)*delta.x + sin(angle-delta.theta)*delta.y;

    temp_F(2,2) = 1;

    Eigen::MatrixXd  F = Eigen::MatrixXd::Zero(3*NUM_LANDMARKS + 3,3*NUM_LANDMARKS + 3);
    Eigen::MatrixXd  eye_temp = Eigen::MatrixXd::Zero(3*NUM_LANDMARKS + 3,3*NUM_LANDMARKS + 3);

    eye_temp(0,0) = 1;
    eye_temp(1,1) = 1;
    eye_temp(2,2) = 1;

    F = eye_temp + Fx.transpose()*temp_F*Fx;
    
    covRobot = F*covRobot*F.transpose() + Fx.transpose()*G*odomNoise*G.transpose()*Fx;
    

}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    // TODO: Write your sensor update procedures here

    for(int i=0;i<observations.size();i++)
    {
        int s = observations[i].markerIndex;
        if(seen[s]==false)
        {
            
            eigRobotState(3*(s+1),0) = eigRobotState(0,0) + observations[i].distance*cos(observations[i].orientation + eigRobotState(2,0));
            eigRobotState(3*(s+1)+1,0) = eigRobotState(1,0) + observations[i].distance*sin(observations[i].orientation + eigRobotState(2,0));
            eigRobotState(3*(s+1)+2,0) = observations[i].orientation;
            seen[s] = true;

        }
        else
        {
            Eigen::Matrix<double,3,1> y;
            y = eigRobotState.segment(3*(s+1),3);

            Eigen::Matrix<double,3,1> zp;
            zp(0) = sqrt(pow(eigRobotState(0)-y(0),2) + pow(eigRobotState(1)-y(1),2));
            zp(1) = atan2((y(1)-eigRobotState(0)),(y(0)-eigRobotState(0))) - eigRobotState(2);

             //wrap angle around
            if(zp(1)>M_PI)
            {
                zp(1) = zp(1) - 2*M_PI;
            }
            else if(zp(1)<-M_PI)
            {
                zp(1) = zp(1) + 2*M_PI;
            }

            
            Eigen::Matrix<double,3,6>temp_H;
            double dx = y(0)-eigRobotState(0);
            double dy = y(1)-eigRobotState(1);
            double q = pow((y(0)-eigRobotState(0)),2) + pow((y(1)-eigRobotState(1)),2);
            double sq = sqrt(q);

            temp_H(0,0) = -(dx)/sq;
            temp_H(0,1) = -(dy)/sq;
            temp_H(0,2) = 0;
            temp_H(0,3) = dx/sq;
            temp_H(0,4) = dy/sq;
            temp_H(0,5) = 0;
            temp_H(1,0 )= dy/q;
            temp_H(1,1) = -dx/q;
            temp_H(1,2) = -1;
            temp_H(1,3 )= -dy/q;
            temp_H(1,4) = dx/q;
            temp_H(1,5) = 0;
            temp_H(2,5 )= 1;

            Eigen::Matrix<double,6,3*NUM_LANDMARKS + 3>Fxj;

            Fxj(0,0) = 1;
            Fxj(1,1) = 1;
            Fxj(2,2) = 1;
            Fxj(3,3*(s+1) )= 1;
            Fxj(4, 3*(s+1)+1) = 1;
            Fxj(5, 3*(s+1)+2) = 1;

            Eigen::Matrix<double,3,3*NUM_LANDMARKS + 3>H;


            H = temp_H*Fxj;
            
            Eigen::Matrix<double,3*NUM_LANDMARKS + 3,3>K;

            K = covRobot*H.transpose()*(((H*covRobot*H.transpose()) + sensorNoise).inverse());

            Eigen::Matrix<double,3,1>innovation;

            innovation = eigRobotState.segment(3*(s+1),3) - zp;

            
            
            eigRobotState += K*innovation;
            Eigen::Matrix<double,3*NUM_LANDMARKS + 3,3*NUM_LANDMARKS + 3>eye;
            eye(0,0) = 1;
            eye(1,1) = 1;
            eye(2,2) = 1;
            
            covRobot = (eye-K*H)*covRobot;

            

        }

    }


    
   
    
}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // TODO: Write your initialization procedures here

   
    eigRobotState(0,0) = robotState.x;
    eigRobotState(1,0) = robotState.y;
    eigRobotState(2,0) = robotState.theta;

    sensorNoise(0,0) = robotParams.sensor_noise_distance;
    sensorNoise(1,1) = robotParams.sensor_noise_distance;
    sensorNoise(2,2) = robotParams.sensor_noise_orientation;
    
    odomNoise(0,0) = robotParams.odom_noise_translation_from_rotation*robotParams.odom_noise_translation_from_translation;
    odomNoise(1,1) = robotParams.odom_noise_translation_from_rotation*robotParams.odom_noise_translation_from_translation;
    odomNoise(2,2) = robotParams.odom_noise_rotation_from_rotation*robotParams.odom_noise_rotation_from_translation;


}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // TODO: Write your drawing procedures here 
    //       (e.g., robot position uncertainty representation)
    
    
//    Example drawing procedure
    // int pixelX, pixelY;
    // double globalX = 1.0, globalY = -1.0;
    // const int NUM_POINTS = 8;
    // const double POINT_SPREAD = 0.2;
    
    // // Draw cyan colored points at specified global locations on field
    // glBegin(GL_POINTS);
    // glColor3f(0.0, 1.0, 1.0);
    // for(int i=0; i<NUM_POINTS; i++){
    //    global2pixel(globalX, globalY + (i * POINT_SPREAD), pixelX, pixelY);
    //    glVertex2i(pixelX, pixelY);
    // }
    // glEnd();

}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here
	
	return 0;
}


/**
 * Main entrypoint for the program.
 */


int main (int argc, char ** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    
    runMainLoop(argc, argv);


    return 0;
}


// To test the localization engine, first compile the program and run it with a robot parameter file:
// make ./localization_test sample_input1.txt
// If the compilation fails, please make sure you have the required OpenGL/glut libraries installed on your
// system.