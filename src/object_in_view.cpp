#include <ros/ros.h>

// Messages
#include <message_filters/subscriber.h>

#include "std_msgs/String.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/cloud_iterator.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/common/transforms.h>

// Camera Model
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

// TF2 Transform includes
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

/**
Converts vector to depth measurement by calculating magnitude
@param xyz vector from camera center to measurement location
@return magnitude of xyz
*/
float absDepthFromVec(cv::Point3d xyz){
    float mag = sqrt(xyz.x*xyz.x + xyz.y*xyz.y + xyz.z*xyz.z);
//    printf("Mag: %2.2f\n", mag);
    return mag;
}

/**
* Find objects in camera FOV
*
* @author Cecilia Mauceri
* @date 26_05_2021
*/
class ObjectInView
{
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    PointCloud cloud;
    FILE * out_f;

public:
    /**
    * callback is called for each incoming image. It transforms a list of object coordinates to camera coordinates
    * and writes to an output file if the object is in FOV and within 10 meters of camera.
    *
    * @param cam_info The camera info for the camera plane which the objects should be transformed to.
    *                   Needs to have an "optical" tf!
    */
    void callback (const CameraInfoConstPtr& cam_info)
    {
        //Transform the point cloud into camera coordinates
        geometry_msgs::TransformStamped transform_msg;
        PointCloud::Ptr cloud_camera(new PointCloud);

        try
        {
            std::string err;
            if(tf_buffer_.canTransform(cam_info->header.frame_id, "chinook/odom",
                              cam_info->header.stamp, ros::Duration(0.1), &err)){
                transform_msg = tf_buffer_.lookupTransform(
                    cam_info->header.frame_id,
                    "chinook/odom",
                    cam_info->header.stamp);

                // Initialize the transformation matrix
                Eigen::Matrix4f tform;
                tform.setIdentity();
                tform(0, 3) = transform_msg.transform.translation.x;
                tform(1, 3) = transform_msg.transform.translation.y;
                tform(2, 3) = transform_msg.transform.translation.z;
                tform.topLeftCorner(3, 3) = Eigen::Matrix3f(Eigen::Quaternionf (transform_msg.transform.rotation.w,
                                                                                    transform_msg.transform.rotation.x,
                                                                                    transform_msg.transform.rotation.y,
                                                                                    transform_msg.transform.rotation.z));
                pcl::transformPointCloud(cloud, *cloud_camera, tform);
//                std::cout << "Got transform " << std::endl;
            }else{
                std::cout << err << std::endl;
                return;
            }
        } catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Filter points in front of camera
        PointCloud::Ptr cloud_filtered(new PointCloud());
        pcl::PassThrough<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud_camera);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0.0, DBL_MAX);
        filter.filter(*cloud_filtered); // z is > 0

//      Camera Model
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info);
        int rows = cam_info->height;
        int cols =  cam_info->width;
        for(auto& point: *cloud_filtered){
             cv::Point3d xyz(point.x, point.y, point.z);
             cv::Point2d uv = cam_model.project3dToPixel(xyz);
             if (uv.x >= 1 && uv.x < cols-1 && uv.y < rows-1 && uv.y >= 1){
                float depth = absDepthFromVec(xyz);
                if(depth < 10.0) {
                    // Object is less than 10 meters away in FOV
                    // Write location of localization point in camera space to file
                    printf("%0.6f %d %d\n", cam_info->header.stamp.toSec(), int(uv.x), int(uv.y));
                    fprintf(out_f, "%0.6f %d %d\n", cam_info->header.stamp.toSec(), int(uv.x), int(uv.y))   ;
                }
             }
        }
    }

    /** Constructor
    @param camera_info message containing camera intrinsics
    @param target_frame string name of output coordinate frame
    */
    ObjectInView(ros::NodeHandle nh, string object_file)
        : tf_listener_(tf_buffer_)
    {
        printf("Constructing node\n");

        out_f = fopen ("output.csv" , "w");
        if (out_f == NULL) perror ("Error opening file");

        //Load object localization points into PointCloud
        cloud.header.frame_id = "chinook/odom";
        cloud.height = cloud.width = 1;

        string line;                    /* string to hold each line */
        ifstream f (object_file);   /* open file */
        if (!f.is_open()) {     /* validate file open for reading */
            perror (("error while opening file " + string(object_file)).c_str());
        }

        while (getline (f, line)) {         /* read each line */
            string val;                     /* string to hold value */
            vector<float> row;                /* vector for row of values */
            stringstream s (line);          /* stringstream to parse csv */
            bool first = true;
            while (getline (s, val, ' ')){   /* for each value */
                if(first){
                    std::cout << val;
                }else{
                    row.push_back (stof(val));  /* convert to float, add to row */
                    printf(" %2.2f", stof(val));
                }
                first = false;
            }
            printf("\n");
            cloud.points.push_back (pcl::PointXYZ(row[0], row[1], row[2]));
        }
        f.close();

        printf ("Node ready\n");
    }

    /** Deconstructor closes output file*/
    ~ObjectInView(){
        fclose(out_f);
    }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_in_view");

  ros::NodeHandle nh;

  ObjectInView x(nh, "/home/ohrad/subt_reference_datasets/data/tunnel/GT/gt_sr_B.csv");

  ros::Subscriber info_sub =nh.subscribe("/chinook/multisense/left/camera_info", 1000, &ObjectInView::callback, &x);
  ros::spin();

  return 0;
}
