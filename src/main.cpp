//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// Copyright (c) 2013, Aldebaran Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

// ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
// ----------------------------------------------------------------------------

#define GL_GLEXT_PROTOTYPES

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <cstdio>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer2d.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>

const std::string DEFAULT_COLOR_FRAME_ID = "camera_rgb_frame";
const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
const std::string OBJECT_FRAME_ID = "obj";
const char *DEPTH_INFO_TOPIC = "/camera/depth/camera_info";
const char *DEPTH_TOPIC = "/camera/depth/image";
const char *COLOR_TOPIC = "/camera/rgb/image_color";
const char *COLOR_INFO_TOPIC = "/camera/rgb/camera_info";
const char *MASK_TOPIC = "/camera/mask";
const char *POSE_TOPIC = "/camera/pose";
const char *TF_TOPIC = "tf";

const double focal_length = 1000;
//const double focal_length_x = 525, focal_length_y = 525;
const double focal_length_x = focal_length, focal_length_y = focal_length; // LJL: these focal points work for the expo marker. These should be configurable...

sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f)
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

sensor_msgs::ImagePtr toImageMsg(std_msgs::Header header, std::string encoding, cv::Mat image)
{
  sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();
  ros_image->header = header;
  ros_image->height = image.rows;
  ros_image->width = image.cols;
  ros_image->encoding = encoding;
  ros_image->is_bigendian = false;
  ros_image->step = image.cols * image.elemSize();
  size_t size = ros_image->step * image.rows;
  ros_image->data.resize(size);

  if (image.isContinuous())
  {
    memcpy((char*)(&ros_image->data[0]), image.data, size);
  }
  else
  {
    // Copy by row by row
    uchar* ros_data_ptr = (uchar*)(&ros_image->data[0]);
    uchar* cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i)
    {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image->step);
      ros_data_ptr += ros_image->step;
      cv_data_ptr += image.step;
    }
  }
  return ros_image;
}

void render3d(std::string file_name, size_t width, size_t height) {
  Renderer3d renderer = Renderer3d(file_name);

  double near = 0.1, far = 1000;

  renderer.set_parameters(width, height, focal_length_x, focal_length_y, near, far);

  RendererIterator renderer_iterator = RendererIterator(&renderer, 10);

  // create bag file, imitates the bag file generated by object_recognition_capture capture
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);

  cv::Rect rect;
  cv::Mat image, depth, mask;

  ros::Time fakeTime = ros::TIME_MIN;
  ros::Duration fakeCapturePeriod(0.2); // 5 FPS for visualization of bag. TODO: this needs to be bigger than synchronization period used in the bag reader if something like that exists
  size_t i;
  for (i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator) {
  //for (size_t i = 0; i < 50; ++i, ++renderer_iterator) { // LJL: testing much faster...
    try {
      renderer_iterator.render(image, depth, mask, rect);
#if 0 // this is the standard output of the renderer
      cv::imwrite(boost::str(boost::format("depth_%05d.png") % (i)), depth);
      cv::imwrite(boost::str(boost::format("image_%05d.png") % (i)), image);
      cv::imwrite(boost::str(boost::format("mask_%05d.png") % (i)), mask);
#endif
      // convert images to image_messages
      std_msgs::Header commonHeader;
      commonHeader.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
      commonHeader.seq = i;
      commonHeader.stamp = fakeTime;
      sensor_msgs::ImagePtr imageMsg = toImageMsg (commonHeader, sensor_msgs::image_encodings::RGB8, image);
      sensor_msgs::ImagePtr depthMsg = toImageMsg (commonHeader, sensor_msgs::image_encodings::TYPE_16UC1, depth);
      sensor_msgs::ImagePtr maskMsg = toImageMsg (commonHeader, sensor_msgs::image_encodings::TYPE_8UC1, mask);
      maskMsg->header =  imageMsg->header;

      // generate pinhole camera_infos
      sensor_msgs::CameraInfoPtr imageInfoMsg = getDefaultCameraInfo(width, height, focal_length_x);
      imageInfoMsg->header = imageMsg->header;
      sensor_msgs::CameraInfoPtr depthInfoMsg = imageInfoMsg;

      // grab the rotation of the camera and convert to a quaternion, TODO: better way to convert basis vector?
      double *x = renderer_iterator.R_obj().val;
      tf::Matrix3x3 cbasis(
              x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8] // is this row wise or column-wise?
      );
      tf::Quaternion q;
      cbasis.inverse().getRotation(q);

      tf2_msgs::TFMessage tfMsg;
      geometry_msgs::TransformStamped cam2ObjMsg;

      // camera -> object
      cam2ObjMsg.header.frame_id = OBJECT_FRAME_ID;
      cam2ObjMsg.child_frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
      cam2ObjMsg.header.seq = i;
      cam2ObjMsg.header.stamp = fakeTime;
      cam2ObjMsg.transform.translation.x =  renderer_iterator.T()[0];
      cam2ObjMsg.transform.translation.y =  renderer_iterator.T()[1];
      cam2ObjMsg.transform.translation.z =  renderer_iterator.T()[2];
      tf::quaternionTFToMsg(q, cam2ObjMsg.transform.rotation);
      tfMsg.transforms.push_back(cam2ObjMsg);

      // reverse the transformation
      tf::StampedTransform cam2obj;
      tf::transformStampedMsgToTF(cam2ObjMsg, cam2obj);
      tf::Transform obj2cam(cam2obj.inverse());

      // generate expected pose stamped of object in camera frame
      geometry_msgs::PoseStamped poseMsg;
      poseMsg.header = commonHeader;
      poseMsg.header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
      poseMsg.pose.position.x = obj2cam.getOrigin().x();
      poseMsg.pose.position.y = obj2cam.getOrigin().y();
      poseMsg.pose.position.z = obj2cam.getOrigin().z();
      tf::quaternionTFToMsg(obj2cam.getRotation(), poseMsg.pose.orientation);

      // write messages to bag
      bag.write(COLOR_TOPIC, fakeTime, imageMsg);
      bag.write(COLOR_INFO_TOPIC, fakeTime, imageInfoMsg);
      bag.write(DEPTH_TOPIC, fakeTime, depthMsg);
      bag.write(DEPTH_INFO_TOPIC, fakeTime, depthInfoMsg);
      bag.write(MASK_TOPIC, fakeTime, maskMsg);
      bag.write(POSE_TOPIC, fakeTime, poseMsg);
      bag.write(TF_TOPIC, fakeTime, tfMsg);

      // increment time
      fakeTime += fakeCapturePeriod;

    } catch (rosbag::BagIOException bexc) {
      ROS_ERROR(bexc.what());
    }
  }
  ROS_INFO("bagged %d views to %s", i, "test.bag");
  // close bag file
  bag.close();
}
/*
void render2d(std::string file_name, size_t width, size_t height) {
  Renderer2d render(file_name, 0.2);
  double focal_length_x = 525, focal_length_y = 525;
  render.set_parameters(width, height, focal_length_x, focal_length_y);
  float y = 0., z = 1;
  cv::Vec2f up(z, -y);
  up = up / norm(up);
  render.lookAt(0.5, y, z, 0, up(0), up(1));
  cv::Mat img, depth, mask;
  cv::Rect rect;
  render.render(img, depth, mask, rect);
  cv::imshow("img", img);
  cv::imshow("depth", depth);
  cv::imshow("mask", mask);
  cv::waitKey(0);
}
*/

// expects mesh file as argument
int main(int argc, char **argv) {
  // Define the display
  //size_t width = 640, height = 480;
  size_t width = 1024, height = 768;

  // the model name can be specified on the command line.
  std::string file_name(argv[1]);
  std::string file_ext = file_name.substr(file_name.size() - 3, file_name.npos);

  // NOTE: model dimensions are expected to be in meters!

  // TODO: the 3d renderer requires the model to be in the current directory, why?
  // simply copy the model into the current directory and delete it after..
  std::string local_file_name = file_name;
  long last_slash_idx = file_name.rfind('\\');
  bool was_copied = false;
  if(last_slash_idx != std::string::npos) {
    local_file_name = file_name.substr(last_slash_idx+1);
    std::ifstream source(file_name.c_str(), std::ios::binary);
    std::ofstream dest(local_file_name.c_str(), std::ios::binary);
    dest << source.rdbuf();
    source.close();
    dest.close();
    was_copied = true;
  }
  //if (file_ext == "png")
    //render2d(file_name, width, height);
  //else
    render3d(local_file_name, width, height);

  if(was_copied){
    remove(local_file_name.c_str());
  }

  return 0;
}
