//
// Created by matt on 4/19/16.
//

#include <ros/ros.h>
#include "can_finder/can_detector.h"

#define TABLE_HEIGHT 1
#define CAN_HEIGHT 1
#define HEIGHT_ERR 0

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_detector"); //node name
    ros::NodeHandle nh;

    PclUtils utils;
    while (!utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    //transform the kinect data to the torso frame;
    // we don't need to have it returned; cwru_pcl_utils can own it as a member var
    utils.transform_kinect_cloud(A_sensor_wrt_torso);
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_transformed_cloud;
    utils.get_kinect_transformed_points(kinect_transformed_cloud);
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(kinect_transformed_cloud); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(TABLE_HEIGHT - HEIGHT_ERR, CAN_HEIGHT + HEIGHT_ERR); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(indices); //  this will return the indices of the points in transformed_cloud_ptr that pass our test
    double num_points = indices.size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_cloud
    for (int i = 0; i < indices.size(); i++) {
        can_cloud.points[i] = kinect_transformed_cloud.points[indices[i]];
    }
#TODO add publisher to publish the can cloud so we can view it in RViz and verify that this is working correctly
}