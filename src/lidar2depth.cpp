#include <ros/ros.h>

// Messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/String.h"
#include <math.h>
#define PI 3.14159265

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/cloud_iterator.h>
#include <pcl/visualization/common/shapes.h>

// Camera Model
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

// Image includes
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// TF2 Transform includes
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace sensor_msgs;
using namespace message_filters;

// How the depth map is encoded in KITTI data:
// Depth maps (annotated and raw Velodyne scans) are saved as uint16 PNG images,
// which can be opened with either MATLAB, libpng++ or the latest version of
// Python's pillow (from PIL import Image). A 0 value indicates an invalid pixel
// (ie, no ground truth exists, or the estimation algorithm didn't produce an
// estimate for that pixel). Otherwise, the depth for a pixel can be computed
// in meters by converting the uint16 value to float and dividing it by 256.0:
//
// disp(u,v)  = ((float)I(u,v))/256.0;
// valid(u,v) = I(u,v)>0;

/**
Converts vector to depth measurement by calculating magnitude and scaling by 256.0
@param xyz vector from camera center to measurement location
@return magnitude of xyz * 256.0
*/
ushort depthFromVec(cv::Point3d xyz){
    float mag = sqrt(xyz.x*xyz.x + xyz.y*xyz.y + xyz.z*xyz.z);
//    printf("Mag: %2.2f\n", mag);
    ushort pixel_val = (ushort)(mag*256.0);
    return pixel_val;
}

/**
Returns a spherical point cloud for testing purposes
@param samples How many points in point cloud
@param center The center of the sphere
@param radius The radius of the sphere
@return PointCloud::Ptr
*/
PointCloud::Ptr fibonacci_sphere(int samples, cv::Point3d center, double radius){
    PointCloud::Ptr cloud(new PointCloud());
    cloud->width  = samples;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    double phi = PI * (3. - sqrt(5.)); //# golden angle in radians

    double i = 0.0;
    for (auto& point: *cloud)
      {
        double  y = 1.0 - (i / (float)(samples - 1)) * 2.0; //  y goes from 1 to -1
        double  y_radius = sqrt(1.0 - y * y) * radius ; // radius at y
        double theta = phi * i; // golden angle increment

        double x = cos(theta) * y_radius;
        double z = sin(theta) * y_radius;

        point.x = x + center.x;
        point.y = y + center.y;
        point.z = z + center.z;
        i++;
      }

    return cloud;
}


/**
* Convert Lidar 3D point cloud to depth map with same image plane as camera
*
* @author Cecilia Mauceri
* @date 26_05_2021
*/
class Lidar2Depth
{
    bool visualize;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    image_transport::Publisher pub_;

public:
    /**
    * cloud_callback is called for each incoming point cloud message. It transforms the point cloud
    * to camera coordinates, and then projects the points on an image plane using a pinhole camera model
    * The value of each projected point is the distance from the point to the image plane producing
    * a depth map. The depth map is then published as a new message.
    *
    * @param cloud_msg The point cloud message
    * @param cam_info The camera info for the camera plane which the point cloud should be transformed to.
    *                   Needs to have an "optical" tf!
    */
    void cloud_callback (const PointCloud2::ConstPtr& cloud_msg,
                         const CameraInfoConstPtr& cam_info)
    {

//      Camera Model
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info);

        //Transform the point cloud into camera coordinates
        geometry_msgs::TransformStamped transform;
        sensor_msgs::PointCloud2 cloud_msg_world;
        try
        {
            transform = tf_buffer_.lookupTransform(
                cam_info->header.frame_id,
                cloud_msg->header.frame_id,
                cloud_msg->header.stamp);
            tf2::doTransform(*cloud_msg, cloud_msg_world, transform);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Convert to PCL data type
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(cloud_msg_world, cloud2);
        PointCloud::Ptr cloud(new PointCloud());

//      // Spherical point cloud for debugging
//        cv::Point3d center(2.0, 2.0, 5.0);
//        PointCloud::Ptr cloud = fibonacci_sphere(1000, center, 1.0);
//
        pcl::fromPCLPointCloud2(cloud2, *cloud);

        // Filter points in front of camera
        PointCloud::Ptr cloud_filtered(new PointCloud);
        pcl::PassThrough<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0.0, DBL_MAX);
        filter.filter(*cloud_filtered); // z is > 0

        // 1. Project each point into image plane
        // 2. Map distance from image plane to pixel value
        // 3. Reshape into image matrix
        int rows = cam_info->height;
        int cols =  cam_info->width;
        cv::Mat image = cv::Mat::zeros(rows, cols, CV_16UC1);

        int max =0;
        for(auto& point: *cloud_filtered){
             cv::Point3d xyz(point.x, point.y, point.z);
             cv::Point2d uv = cam_model.project3dToPixel(xyz);
             if (uv.x >= 1 && uv.x < cols-1 && uv.y < rows-1 && uv.y >= 1){
                int depth = depthFromVec(xyz);
                if(depth > max) max = depth;

                image.at<ushort>((int)uv.y, (int)uv.x) = depth;

                //Make lidar points bigger for visualization
                if(visualize){
                    for(int j=-1; j<2; j++){
                        image.at<ushort>((int)uv.y+j, (int)uv.x) = depth;
                        image.at<ushort>((int)uv.y, (int)uv.x+j) = depth;
                        image.at<ushort>((int)uv.y+j, (int)uv.x+j) = depth;
                    }
                }
             }
        }

        // Scale values to max for visualization
        if(visualize){
            int scale = 65535/max;
            image *= scale;
        }

        // Convert to ROS data type, copy the header from cloud_msg
        std_msgs::Header header = cam_info-> header;
        sensor_msgs::ImagePtr output = cv_bridge::CvImage(header, "mono16", image).toImageMsg();

        // Publish the data
        pub_.publish (output);
    }

    /** Constructor
    @param camera_info message containing camera intrinsics
    @param target_frame string name of output coordinate frame
    */
    Lidar2Depth(ros::NodeHandle nh)
        : tf_listener_(tf_buffer_)
    {
        printf("Constructing node\n");

        // Create a ROS publisher for the output depth image
        image_transport::ImageTransport it(nh);
        pub_ = it.advertise("/depth_image", 1);

        // Flag whether to enhance visualization
        nh.param("visualize", visualize, false);
        if(visualize)
            printf("Visualization mode on\n");

        printf ("Node ready\n");
    }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "lidar2depth_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<PointCloud2> image_sub(nh, "/points", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera_info", 1);
  TimeSynchronizer<PointCloud2, CameraInfo> sync(image_sub, info_sub, 10);

  Lidar2Depth x(nh);
  sync.registerCallback(boost::bind(&Lidar2Depth::cloud_callback, &x, _1, _2));

  ros::spin();

  return 0;
}
