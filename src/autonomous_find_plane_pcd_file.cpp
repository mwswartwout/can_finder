//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(),
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile()
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include<ros/ros.h>
//#include <stdlib.h>
//#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>
//#include <pcl-1.7/pcl/point_cloud.h>
//#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs

using namespace std;
PclUtils *g_pcl_utils_ptr;

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr patch_cloud_ptr,
                                      vector<int> &indices) {

    float curvature;
    Eigen::Vector4f plane_parameters;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    ROS_INFO_STREAM( "PCL: plane params of patch: " << plane_parameters.transpose() );

    //next, define a coordinate frame on the plane fitted to the patch.
    // choose the z-axis of this frame to be the plane normal--but enforce that the normal
    // must point towards the camera
    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
    ROS_INFO_STREAM( "A_plane_wrt_camera rotation:" );
    ROS_INFO_STREAM( A_plane_wrt_camera.linear() );
    ROS_INFO_STREAM( "origin: " << A_plane_wrt_camera.translation().transpose() );

    //next, transform all points in input_cloud into the plane frame.
    //the result of this is, points that are members of the plane of interest should have z-coordinates
    // nearly 0, and thus these points will be easy to find
    ROS_INFO_STREAM( "transforming all points to plane coordinates..." );
    //Transform each point in the given point cloud according to the given transformation.
    //pcl fnc: pass in ptrs to input cloud, holder for transformed cloud, and desired transform
    //note that if A contains a description of the frame on the plane, we want to xform with inverse(A)
    pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());

    //now we'll use some functions from the pcl filter library;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(float(-0.02), 0.02); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
    ROS_INFO_STREAM( "number of points passing the filter = " << indices.size() );
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
    pcl::PointXYZ origin(0, 0, 0);
    pcl::PointXYZ focal_point = pcl::transformPoint(origin, A_plane_wrt_camera.inverse());
    ROS_INFO_STREAM("Focal point in object coordinates is [ " << focal_point.x << ", " << focal_point.y << ", " << focal_point.z << " ].");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stool_plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stool_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> indices;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages
    string fname;
    ROS_INFO_STREAM("enter pcd file name: "); //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("pcd", 1);
    ros::Publisher pubPlaneStool = nh.advertise<sensor_msgs::PointCloud2> ("stool_planar_pts", 1);
    ros::Publisher pubPlaneCan = nh.advertise<sensor_msgs::PointCloud2> ("can_planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);

    sensor_msgs::PointCloud2 ros_cloud, ros_stool_planar_cloud, ros_can_planar_cloud, downsampled_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    ROS_INFO_STREAM( "starting voxel filtering" );
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    ROS_INFO_STREAM( "done voxel filtering" );

    ROS_INFO_STREAM( "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() );
    ROS_INFO_STREAM( "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() ); // ->data.size()<<endl;
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

    // Find the stool
    double x_min = -.05;
    double x_max = .3;
    double y_min = -.2;
    double y_max = .2;
    double z_min = .5;
    double z_max = .8;

    for (int i = 0; i < downsampled_kinect_ptr->points.size(); i++) {
        float x = downsampled_kinect_ptr->points[i].x;
        float y = downsampled_kinect_ptr->points[i].y;
        float z = downsampled_kinect_ptr->points[i].z;
        bool x_valid = (x < x_max) && (x > x_min);
        bool y_valid = (y < y_max) && (y > y_min);
        bool z_valid = (z < z_max) && (z > z_min);

        if (x_valid && y_valid && z_valid) {
            stool_cloud_ptr->push_back(downsampled_kinect_ptr->points[i]);
        }
    }

    //find pts coplanar w/ selected patch, using PCL methods in above-defined function
    //"indices" will get filled with indices of points that are approx co-planar with the selected patch
    // can extract indices from original cloud, or from voxel-filtered (down-sampled) cloud
    //find_indices_of_plane_from_patch(pclKinect_clr_ptr, stool_cloud_ptr, indices);
    find_indices_of_plane_from_patch(downsampled_kinect_ptr, stool_cloud_ptr, indices);
    pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *stool_plane_pts_ptr); //extract these pts into new cloud
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*stool_plane_pts_ptr, ros_stool_planar_cloud); //convert to ros message for publication and display

    // Find the can top
    x_min = .04;
    x_max = .14;
    y_min = 0;
    y_max = .06;
    z_min = .53;
    z_max = .62;

    for (int i = 0; i < downsampled_kinect_ptr->points.size(); i++) {
        float x = downsampled_kinect_ptr->points[i].x;
        float y = downsampled_kinect_ptr->points[i].y;
        float z = downsampled_kinect_ptr->points[i].z;
        bool x_valid = (x < x_max) && (x > x_min);
        bool y_valid = (y < y_max) && (y > y_min);
        bool z_valid = (z < z_max) && (z > z_min);

        if (x_valid && y_valid && z_valid) {
            can_cloud_ptr->push_back(downsampled_kinect_ptr->points[i]);
        }
    }

    Eigen::Vector3f centroid = g_pcl_utils_ptr->compute_centroid(can_cloud_ptr);
    ROS_INFO_STREAM("Centroid of top of can is " << centroid[0] << ", " << centroid[1] << ", " << centroid[2]);
    can_cloud_ptr->header.frame_id="camera_depth_optical_frame";
    pcl::toROSMsg(*can_cloud_ptr, ros_can_planar_cloud);

    while (ros::ok()) {
        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlaneStool.publish(ros_stool_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubPlaneCan.publish(ros_can_planar_cloud);
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }
    return 0;
}
