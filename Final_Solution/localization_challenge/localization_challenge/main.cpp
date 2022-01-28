#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"
#include <iostream>
/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
RobotState odo_global, sensor_error;
FieldLocation reference_point[NUM_LANDMARKS];
RobotParams error_matrix;


void getRobotPositionEstimate(RobotState& estimatePosn)
{
    // TODO: Write your procedures to set the current robot position estimate here
    
    estimatePosn.x = odo_global.x - 0.015 * sensor_error.x ;
    estimatePosn.y = odo_global.y - 0.015 * sensor_error.y;
    estimatePosn.theta = odo_global.theta - 0.015 * sensor_error.theta;
    
    ////  wrap angle around
    if(estimatePosn.theta>M_PI)
    {
        estimatePosn.theta = estimatePosn.theta - 2*M_PI;
    }
    else if(estimatePosn.theta<-M_PI)
    {
        estimatePosn.theta = estimatePosn.theta + 2*M_PI;
    }

    odo_global = estimatePosn;
    sensor_error.x = 0;
    sensor_error.y = 0;
    sensor_error.theta = 0;

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
    (odo_global).x = (odo_global).x + delta.x * cos((odo_global).theta) - sin((odo_global).theta) * delta.y;
    (odo_global).y = (odo_global).y +  sin((odo_global).theta) * delta.x + cos((odo_global).theta) * delta.y;
    (odo_global).theta = (odo_global).theta + delta.theta;
    
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
    std::vector<RobotState> error;
    
    int size = observations.size();

    // TODO: Write your sensor update procedures here
    for (int i = 0; i < size ; i++ )
    {   
        error.push_back(RobotState());
        int j = observations[i].markerIndex;

        // if ((odo_global.theta < 180) && (odo_global.theta < -180))
        // {
        //     std::cout<<"\n global theta "<<odo_global.theta*180/3.142<<"\t";
        // }

        error_matrix.sensor_noise_distance;
        error_matrix.sensor_noise_orientation; 
        double orientation_calculated = (atan2((reference_point[j].y - odo_global.y), (reference_point[j].x - odo_global.x))) - odo_global.theta; 
        double error_bearing = observations[i].orientation - orientation_calculated;
        double d_dash = (sqrt(pow(( odo_global.x - reference_point[i].x ), 2)+pow((odo_global.y - reference_point[i].y), 2)));
        error[i].x = d_dash * sin(orientation_calculated) - observations[i].distance * sin(observations[i].orientation);
        error[i].y = d_dash * cos(orientation_calculated) - observations[i].distance * cos(observations[i].orientation);
        error[i].theta = error_bearing;

    }

    double x = 0;
    double y = 0;
    double theta = 0;
    
    for(int k = 0; k<size; k++)
    {
        x = x + error[k].x;
        y = y + error[k].y;
        theta = theta + error[k].theta;
    }
    
    x = x/size;
    y = y/size;
    theta = theta/size; 
    (sensor_error).x = x;
    (sensor_error).y = y;
    (sensor_error).theta = theta;
    // std::cout<<"\nsensor error x "<< (sensor_error).x;
    // std::cout<<"\t sensor error y "<< (sensor_error).y;
    // std::cout<<"\tsensor error theta "<< (sensor_error).theta;
    
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
    (odo_global).x = robotState.x;
    (odo_global).y = robotState.y;
    (odo_global).theta = robotState.theta;
    for(int i=0;i<NUM_LANDMARKS; i++)
    {
        reference_point[i] = markerLocations[i];

    }
    error_matrix = robotParams;
 
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
    
    
//    // Example drawing procedure
    int pixelX, pixelY;
    double globalX = 1.0, globalY = -1.0;
    const int NUM_POINTS = 8;
    const double POINT_SPREAD = 0.2;
    
    // Draw cyan colored points at specified global locations on field
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 1.0);
    for(int i=0; i<NUM_POINTS; i++){
       global2pixel(globalX, globalY + (i * POINT_SPREAD), pixelX, pixelY);
       glVertex2i(pixelX, pixelY);
    }
    glEnd();

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

