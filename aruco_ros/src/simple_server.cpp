/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <sensor_msgs/Image.h>
#include <aruco_msgs/MarkersReq.h>


class ArucoSimpleServer
{
private:
  cv::Mat inImage;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  bool cam_info_received;
  
  ros::ServiceServer serverMarker;


  ros::NodeHandle nh;



public:
  ArucoSimpleServer() :
     nh("~")
  {
    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);


    serverMarker = nh.advertiseService("aruco", &ArucoSimpleServer::callbackServerMarker, this);
  }



  bool callbackServerMarker(aruco_msgs::MarkersReq::Request  &req, aruco_msgs::MarkersReq::Response &res)
  {
    res.markers.clear();
    aruco::CameraParameters camParam = aruco_ros::rosCameraInfo2ArucoCamParams(req.info, req.isRectified.data);
    static tf::TransformBroadcaster br;
    
    ros::Time curr_stamp = req.image.header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, req.markerSize.data, false);
        // for each marker, draw info and its boundaries in the image
        for(int i = 0; i < req.ids.size(); i++)
        {
            for(int k = 0; k < markers.size(); k++)
            {
                if (req.ids[i].data == markers[k].id)
                {
                    tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[k]);
                    std::string marker_frame = "marker_" + std::to_string(markers[k].id);

                    tf::StampedTransform stampedTransform(transform, curr_stamp, req.image.header.frame_id, marker_frame);
                    br.sendTransform(stampedTransform);
                    geometry_msgs::PoseStamped poseMsg;
                    tf::poseTFToMsg(transform, poseMsg.pose);
                    aruco_msgs::Marker tmpMarker;
                    
                    tmpMarker.header.frame_id = req.image.header.frame_id;
                    tmpMarker.header.stamp = curr_stamp;
                    tmpMarker.pose.pose = poseMsg.pose;
                    tmpMarker.id = markers[k].id;
                    tmpMarker.point.x = markers[k].getCenter().x;
                    tmpMarker.point.y = markers[k].getCenter().y;
                    tmpMarker.point.z = 0;
                    for(int n= 0;n < 4; n++)
                    {
                      geometry_msgs::Point tmp_pixel;
                      tmp_pixel.x = markers[k][n].x;
                      tmp_pixel.y = markers[k][n].y;
                      tmp_pixel.z = 0;
                      tmpMarker.pixels.push_back(tmp_pixel);

                    }
                    // tmpMarker.pixel_x = markers[k][0].x;
                    // int testy = markers[k][0].y;
                    // tmpMarker.pixel.x = markers[k].getCenter().x;
                    // tmpMarker.pixel.y = markers[k].getCenter().y;
                    // tmpMarker.pixel.z = 0;
                    res.markers.push_back(tmpMarker); 
                    //std::cout<< "pose: " << poseMsg.pose << std::endl;
                    break;
                }
            }
        }
        return true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple_server");

  ArucoSimpleServer node;

  ros::spin();
}
