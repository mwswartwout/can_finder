//
// Created by matt on 4/19/16.
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <pcl_utils/pcl_utils.h>
#include <pcl/filters/passthrough.h>

#define TABLE_HEIGHT .82
#define CAN_HEIGHT .95
#define TABLE_HEIGHT_ERR 0.1
#define CAN_HEIGHT_ERR 0.05
#define MIN_CLOUD_SIZE 100

tf::StampedTransform wait_for_transform() {
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
            tf_listener.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    return tf_sensor_frame_to_torso_frame;
}

bool detect_can(ros::NodeHandle& nh, PclUtils& utils, tf::StampedTransform tf_sensor_frame_to_torso_frame) {
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    //transform the kinect data to the torso frame;
    // we don't need to have it returned; pcl_utils can own it as a member var
    ROS_INFO("Transforming kinect cloud");
    utils.transform_kinect_clr_cloud(A_sensor_wrt_torso);

    // Save transformed kinect data into PointCloud object that we can manipulate
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ROS_INFO("Getting transformed kinect cloud");
    utils.get_kinect_transformed_points(kinect_transformed_cloud);

    // Filter the kinect cloud to just contain points that could feasibly be a part of the can
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(kinect_transformed_cloud); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(TABLE_HEIGHT - TABLE_HEIGHT_ERR, CAN_HEIGHT + CAN_HEIGHT_ERR); //here is the range of z values
    std::vector<int> indices;
    ROS_INFO("Filtering cloud by z height");
    pass.filter(indices); //  this will return the indices of the points in transformed_cloud_ptr that pass our test
    ROS_INFO_STREAM( indices.size() << " indices passed by z filter.");
    // Create can cloud just with points that are the can
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < indices.size(); i++) {
        can_cloud->points[i] = kinect_transformed_cloud->points[indices[i]];
    }
    // TODO Could add color filtering in addition to spatial

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("can", 1, true);
    sensor_msgs::PointCloud2 ros_can_cloud;
    pcl::toROSMsg(*can_cloud, ros_can_cloud);
    pubCloud.publish(ros_can_cloud); // will not need to keep republishing if display setting is persistent
    ros::spinOnce();

    bool found;
    if (can_cloud->points.size() > MIN_CLOUD_SIZE) {
        ROS_INFO("Can detected at this position");
        ROS_INFO_STREAM("Can cloud has size " << can_cloud->points.size());
        found = true;
    }
    else {
        ROS_INFO("No can is present");
        ROS_INFO_STREAM("Can cloud has size " << can_cloud->points.size());
        found = false;
    }

    while (ros::ok) {
        pubCloud.publish(ros_can_cloud); // will not need to keep republishing if display setting is persistent
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return found;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_detector"); //node name
    ros::NodeHandle nh;

    PclUtils utils(&nh);
    while (!utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    tf::StampedTransform tf_sensor_frame_to_torso_frame = wait_for_transform();

    bool found = detect_can(nh, utils, tf_sensor_frame_to_torso_frame);

    return 0;
}