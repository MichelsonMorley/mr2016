#include "tuw_local_planner/tuw_local_planner.h"
#include <opencv/cv.h>
#include <boost/concept_check.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace tuw;

std::map<LocalPlanner::ControlMode, std::string> LocalPlanner::ControlModeName_ = {
    {DEMO, "DEMO"},
    {STOP, "STOP"},
    {WANDERER, "WANDERER"},
    {WANDERER2, "WANDERER2"},
    {WALL_FOLLOWING, "WALL_FOLLOWING"},
    {WALL_FOLLOWING2, "WALL_FOLLOWING2"},
    {GOTO, "GOTO"},
    {GOTO2, "GOTO2"}
};

LocalPlanner::LocalPlanner ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Local View" ) {

}

void LocalPlanner::init() {

    figure_local_.init ( config_.map_pix_x, config_.map_pix_y,
                         config_.map_min_x, config_.map_max_x,
                         config_.map_min_y, config_.map_max_y,
                         config_.map_rotation + M_PI/2.0,
                         config_.map_grid_x, config_.map_grid_y );
    



    cv::putText ( figure_local_.background(), ControlModeName_[ ( ControlMode ) config_.mode],
                  cv::Point ( 5,figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA );

    /**
     * @ToDo Wanderer
     * changes the Maxima Musterfrau to your name
     **/
    cv::putText ( figure_local_.background(), "Adrian Schoisengeier",
                  cv::Point ( figure_local_.view().cols-250, figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA );
}

void LocalPlanner::plot() {
    if ( config_.plot_data ) plotLocal();
    cv::waitKey ( 10 );
}

void LocalPlanner::plotLocal() {
    figure_local_.clear();
    for ( size_t i = 0; i < measurement_laser_.size(); i++ ) {
        /**
        * @ToDo Wanderer
        * uses Figure::symbol or Figure::circle to plot the laser measurements in a for loop
        **/
	
	figure_local_.symbol( figure_local_.view(), measurement_laser_[i].end_point, Figure::red);
    }
    cv::imshow ( figure_local_.title(),figure_local_.view() );
}

void LocalPlanner::ai() {
    if ( measurement_laser_.empty() ) {
        cmd_.set ( 0, 0 );
        return;
    }
    switch ( config_.mode ) {
    case STOP:
        cmd_.set ( 0, 0 );
        break;
    case DEMO:
        ai_demo();
        break;
    case WANDERER:
        ai_wanderer();
        break;
    case WANDERER2:
        ai_wanderer2();
        break;
    default:
        cmd_.set ( 0, 0 );
    }
    loop_count_++;
}

/**
* Demo
**/
void LocalPlanner::ai_demo() {
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.2, w = -0.02;
    } else {
        if ( measurement_laser_[measurement_laser_.size() / 4].length < 1.0 ) {
            w = 0.4;
        } else {
            v = 0.4;
        }
    }
    cmd_.set ( v, w );
}

/**
* @ToDo Wanderer
* writes one or two wanderer behaviours to keep the robot at least 120sec driving without a crash by exploring as much as possible.
* I know it sounds weird but don't get too fancy. 
**/
void LocalPlanner::ai_wanderer() {
  
  
    double v = 0.8, w = 0.0;
    
    double front_right = measurement_laser_[2* measurement_laser_.size() / 5].length;
    double front_left = measurement_laser_[3* measurement_laser_.size() / 5].length;
    double side_right = measurement_laser_[1* measurement_laser_.size() / 5].length;
    double side_left = measurement_laser_[4* measurement_laser_.size() / 5].length;
    
    bool danger;
    
        
    if ( measurement_laser_.empty() ) {
        v = 0.2, w = -0.02;
	cmd_.set ( v, w );
	return;
    } 
    
    // VERY CLOSE OBJECTD
    
    for (int i=0; i<measurement_laser_.size() ; i++) {
      
      if ( measurement_laser_ [i].length < 0.35 ) {
	v = 0.0; w = 0.5;
	cmd_.set ( v, w );
	return;
      }
    }
   
    if ( front_right < 1.0 ) {
            w = 0.5;
	    v = 0.8/0.75 * ( front_right - 0.25);
    } 
    else if ( front_left < 1.0 ) {
            w = -0.5;
	    v = 0.8/0.75 * (front_left - 0.25);
    }
    else if ( side_right < 0.4 ) {
            w = 0.5;
	    v = 0.8/0.15 * (side_right - 0.23);     
    } 
    else if ( side_left < 0.4 ) {
            w = -0.5;
	    v = 0.8/0.15 * (side_left -0.23);
    } 
    else {
            v = 0.8;
	    w = 0.0;
    }
    
    cmd_.set ( v, w );
    
    //ROS_INFO ("Velocity: %f", v);
    //ROS_INFO ("Rotation speed: %f", w);
}
/**
* @ToDo Wanderer
* OPTIONAL: if you like you can write a other one :-)
**/
void LocalPlanner::ai_wanderer2() {
  
  
    double v = 0.8, w = 0.0;
    
    double front_right = measurement_laser_[4* measurement_laser_.size() / 10].length;
    double front_left = measurement_laser_[6* measurement_laser_.size() / 10].length;
    double side_right = measurement_laser_[2* measurement_laser_.size() / 10].length;
    double side_left = measurement_laser_[8* measurement_laser_.size() / 10].length;
    
    bool danger;
    
        
    if ( measurement_laser_.empty() ) {
        v = 0.2, w = -0.02;
	cmd_.set ( v, w );
	return;
    } 
    
    // VERY CLOSE OBJECTD
    
    for (int i=0; i<measurement_laser_.size() ; i++) {
      
      if ( measurement_laser_ [i].length < 0.35 ) {
	v = 0.0; w = 0.5;
	cmd_.set ( v, w );
	return;
      }
    }
    
    if ( front_right < 1.5 ) {
            w = 0.5;
	    v = 0.8 * sqrt(front_right/1.5);
    } 
    else if ( front_left < 1.5 ) {
            w = -0.5;
	    v = 0.8/0.75 * sqrt(front_left/1.5);
    }
    else if ( side_right < 0.4 ) {
            w = 0.5;
	    v = 0.8;     
    } 
    else if ( side_left < 0.4 ) {
            w = -0.5;
	    v = 0.8;
    } 
    else {
            v = 0.8;
	    w = 0.0;
    }
    
    cmd_.set ( v, w );
  
}
