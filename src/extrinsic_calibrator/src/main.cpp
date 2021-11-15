
#include "extrinsic_calibrator.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init (argc, argv, "point_cloud_aligner_node");

    ros::NodeHandle nh;

    PointCloudAligner point_cloud_aligner(nh);

    ros::spin ();
    ros::waitForShutdown();
    return 0;
}
