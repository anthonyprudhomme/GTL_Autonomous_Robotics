
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Joy.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;

        // This might be useful
        double radius;
		double side_threshold;
		double min_distance_threshold;
		double max_distance_threshold;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

		geometry_msgs::Twist desired;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
			desired = *msg;
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
			geometry_msgs::Twist res = desired;
			
			unsigned int n = lastpc.size();
			std::vector<size_t> pidx;
			// filter points
	    	for (unsigned int i=0;i<n;i++) {
                float x = lastpc[i].x;
                float y = lastpc[i].y;
                float d = hypot(x,y);
                if (d > radius) {
                    // too far, ignore
                    continue;
                }
				if (d < 0.05) {
                    // too far, ignore
                    continue;
                }
				if(fabs(y) > side_threshold){
					// too far on the side
                    continue;
				}
				if(x < 0){
					// point is behind the robot
                    continue;
				}
                pidx.push_back(i);
            }
            //ROS_INFO("New point cloud: %d points",n);

			double min_distance = 100;
	    	for (unsigned int i=0; i < pidx.size(); i++) {
				pcl::PointXYZ current_point = lastpc[pidx[i]];
                float x = current_point.x;
                float y = current_point.y;
                float z = current_point.z;
                //ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
				double distance = hypot(x,y);
				if(min_distance > distance){
					min_distance = distance;
					//ROS_INFO("New Min dist: %f", distance);
				}
            }
			//ROS_INFO("Min dist: %f, thresh_min: %f", min_distance, min_distance_threshold);
			if(min_distance < min_distance_threshold){
				ROS_INFO("Stop");
				if(res.linear.x > 0){
					res.linear.x = 0;
				}
			}else{
				if(min_distance > min_distance_threshold && min_distance < max_distance_threshold){
					ROS_INFO("Slow down");
					if(res.linear.x > 0){
						res.linear.x = (min_distance - min_distance_threshold)/(max_distance_threshold-min_distance_threshold)*res.linear.x;
					}
				}else{
					ROS_INFO("Continue");			
				}
			}
            printf("\n");

			velPub.publish(res);
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
			nh.param("side_threshold",side_threshold,1.0);
			nh.param("min_distance_threshold",min_distance_threshold,0.5);
			nh.param("max_distance_threshold",max_distance_threshold,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


