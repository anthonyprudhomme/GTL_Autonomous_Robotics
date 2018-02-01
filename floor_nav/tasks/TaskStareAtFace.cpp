#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

double target;

TaskIndicator TaskStareAtFace::initialise() 
{
    //ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
	const face_detect_base::ROIArray faces = env->getFacesDetectedDuringTask();
	double x_offset = faces.ROIArray[0].x_offset;
	double width = faces.ROIArray[0].width;
	double face_center_x = x_offset + width/2;
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    initial_heading = tpose.theta;
	ROS_INFO("initial_heading %.2f ",initial_heading);
	target =(((128-face_center_x)/256)*(M_PI/180))*50;
	ROS_INFO("target %.2f ",target);
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double alpha = remainder(initial_heading+target-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_theta*alpha;
    if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
