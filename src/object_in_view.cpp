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
#include <eigen_conversions/eigen_msg.h>
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
#include <opencv2/core/mat.hpp>

// TF2 Transform includes
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

/**
Converts vector to depth measurement by calculating magnitude
@param xyz vector from camera center to measurement location
@return magnitude of xyz
*/
double absDepthFromVec(cv::Point3d xyz){
    double mag = sqrt(xyz.x*xyz.x + xyz.y*xyz.y + xyz.z*xyz.z);
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
    int count;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<std::string> object_names;
    std::vector<tf2::Vector3> cloud;
    FILE * out_f;

public:
    /**
    * callback is called for each incoming image. It transforms a list of object coordinates to camera coordinates
    * and writes to an output file if the object is in FOV and within 10 meters of camera.
    *
    * @param cam_info The camera info for the camera plane which the objects should be transformed to.
    *                   Needs to have an "optical" tf!
    */
    void callback (const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
//        std::cout << "Start callback" << std::endl;
        //Transform the point cloud into camera coordinates
        geometry_msgs::TransformStamped transform_msg;
        PointCloud::Ptr cloud_camera(new PointCloud);

        try
        {
            std::string err;
            if(tf_buffer_.canTransform(cam_info->header.frame_id, "chinook/odom",
                              cam_info->header.stamp, ros::Duration(0.1), &err)){
//                std::cout << cam_info->header.frame_id << std::endl;
                transform_msg = tf_buffer_.lookupTransform(
                    cam_info->header.frame_id,
                    "chinook/odom",
                    cam_info->header.stamp);
            }else{
                std::cout << err << std::endl;
                return;
            }
        } catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

//      Camera Model
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(cam_info);

        int rows = cam_info->height;
        int cols =  cam_info->width;
        bool written = false;
        for(int ii =0 ; ii<cloud.size(); ii++){
             const tf2::Vector3 point = cloud[ii];
             string name = object_names[ii];

             geometry_msgs::Point point_msg_camera, point_msg;
             tf2::toMsg(point, point_msg);
//             printf("Point tf2: %0.2f %0.2f %0.2f\n", point_msg.x, point_msg.y, point_msg.z);
             tf2::doTransform(point_msg, point_msg_camera, transform_msg);

//             printf("Point tf2: %0.2f %0.2f %0.2f\n", point_msg_camera.x, point_msg_camera.y, point_msg_camera.z);
             cv::Point3d xyz(point_msg_camera.x, point_msg_camera.y, point_msg_camera.z);
//              printf("Point cv: %0.2f %0.2f %0.2f\n", xyz.x, xyz.y, xyz.z);
             cv::Point2d uv = cam_model.unrectifyPoint(cam_model.project3dToPixel(xyz));
//             printf("Point proj: %0.2f %0.2f\n", uv.x, uv.y);
             if (uv.x >= 1 && uv.x < cols-1 && uv.y < rows-1 && uv.y >= 1 && xyz.z > 0){
                double depth = absDepthFromVec(xyz);
                if(depth < 10.0) {
//                    printf("Translation: %0.2f %0.2f %0.2f\n", transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);

                    // Object is less than 10 meters away in FOV
                    // Write location of localization point in camera space to file
                    double timeStamp = cam_info->header.stamp.toSec();
                    printf("%s %0.6f %d %d %f\n", name.c_str(), timeStamp, int(uv.x), int(uv.y), depth);
//                     fprintf(out_f, "object\tworld_x\tworld_y\tworld_z\ttimestamp\tcam_x\tcam_y\n")   ;
                    fprintf(out_f, "%s\t%0.2f\t%0.2f\t%0.2f\t%0.6f\t%d\t%d\n", name.c_str(),
                                point_msg.x, point_msg.y, point_msg.z,
                                timeStamp, int(uv.x), int(uv.y))   ;

                    if (!written)
                        written = this->writeImage(image, timeStamp);

                }
             }
        }
    }

    bool writeImage(const sensor_msgs::ImageConstPtr& image_msg, double timeStamp){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        // Save as png image
        char *buffer = new char[1000];
        std::string fmt = "img%0.8f.png";
        int n = sprintf(buffer, fmt.c_str(), timeStamp);
        assert (n >= 0 and n < 1000 - 1  && "check fmt_str output");
        std::string filepath (buffer);
        delete buffer;

        cv::imwrite( filepath, cv_ptr->image );
        std::cout << "Saved " << filepath << std::endl;
        return true;
    }

    /** Constructor
    @param camera_info message containing camera intrinsics
    @param target_frame string name of output coordinate frame
    */
    ObjectInView(ros::NodeHandle nh, string object_file)
        : tf_listener_(tf_buffer_)
    {
        printf("Constructing node\n");
        count = 0;

        out_f = fopen ("output.csv" , "w");
        if (out_f == NULL) perror ("Error opening file");
        fprintf(out_f, "object\tworld_x\tworld_y\tworld_z\ttimestamp\tcam_x\tcam_y\n")   ;

        //Load object localization points into PointCloud

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
                    object_names.push_back(val);
                }else{
                    row.push_back (stof(val));  /* convert to float, add to row */
                    printf(" %2.2f", stof(val));
                }
                first = false;
            }
            printf("\n");
            cloud.push_back (tf2::Vector3(row[0], row[1], row[2]));
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

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints("compressed");
  image_transport::CameraSubscriber sub = it.subscribeCamera("/chinook/multisense/left/image_rect_color", 10,
                                                            &ObjectInView::callback, &x, hints);

  ros::spin();

  return 0;
}
