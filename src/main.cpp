/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-14 20:15
 */

#include <lidar_lane_detection/lane_detection.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "lane_detection");

    LaneDetector LD;
    
    ros::spin();
}