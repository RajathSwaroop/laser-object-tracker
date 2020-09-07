#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;

/*

find and track the bucket using laser data.

known params - diameter of bucket - 280mm, measurement is in meter scale

Algorithm
1) Project the laser sensor reading on an image at a higher scale (1m to 50m) scale in this program
2) Use canny edge detector to detect edges in the image to detect the bucket projected as semi circle
3) Use HoughTransform algorithm to find the circles in the image, bucket is a circle with 0.28 * 50 / 2 = 7pixels of radius hence we use min radius 6 and max radius 8
4) Use kalman filter with dimension of state = 4 [ x, y, v_x, v_y ] and measurement dimension = 2 [ x, y] to predict the position of the center of circle.

Assumptions:
1) Assumed the hough transform can detect the semi circle representation of the bucket accurately. (works good with car straight bag file)
2) Assumed and developed for a case where only one bucket or object is moving around.

Can Improve:
1) Could make the state of KF [x, theta, vel, d_theta] and use extended kalman filter for better pose prediction
2) Could use buckets using cv::Mat to eliminate wall from the laser points

*/

struct object{
  cv::Point center;
  double distance;
  double theta;
  double velTheta;
  double vel;
  double radius;
  double vx;
  double vy;
  double distDiff = DBL_MAX;
};

struct imageParams{
  int rows;
  int cols;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double scale = 1.0;
};
//ros::Publisher pub_image;
image_transport::Publisher pub_image;
vector<object> past_objects;
ros::Time pastTime;
ros::Publisher pub_cloud;
ros::Publisher pub_bucket_data;
long found = 0;
long cb = 0;
bool firstExecution = true;
bool publishImage = false;
bool displayImage = false;

// compute euclidean distance between two points in x,y 2d space
double euclidean_distance(const cv::Point& p1, const cv::Point& p2)
{
  return sqrt( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) );
}

int stateDim=4;
int measDim=2;
int ctrlDim=0;

cv::KalmanFilter KF(stateDim,measDim,ctrlDim,CV_32F);
cv::Mat state(stateDim,1,CV_32F);
cv::Mat_<float> measurement(2,1); 

bool corrected = false;

// initialize Kalman filter
void initKF()
{
  float sigmaP=0.01;
  float sigmaQ=0.1;

  cv::setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(sigmaP));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(sigmaQ));//1e-1

}

// correction or update step of Kalman filter
void correctKF(const cv::Point2f& center, const cv::Point2f& vel)
{
  static cv::Mat measMat = cv::Mat( 2, 1, CV_32F);
  measMat.at<float>(0,0) = center.x;
  measMat.at<float>(1,0) = center.y;
  
  KF.correct( measMat );
  corrected = true;
}

// get the bucket position from image
cv::Point2f getBucketPos(const imageParams& param, const object& bucket )
{
  cv::Point2f bucketPos = { bucket.center.x, bucket.center.y };
  bucketPos.x = ( bucketPos.x - param.offset_x ) / param.scale;
  bucketPos.y = ( bucketPos.y - param.offset_y ) / param.scale;
  return bucketPos;
}

// publish the bucket position as odometry msg
void publish_odometry(const object& bucket, const imageParams& param)
{
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  cv::Point2f bucketPos = getBucketPos( param, bucket );
  msg.pose.pose.position.x = bucketPos.x;
  msg.pose.pose.position.y = bucketPos.y;
  msg.twist.twist.linear.x = bucket.vx / param.scale;
  msg.twist.twist.linear.y = bucket.vy / param.scale;
  pub_bucket_data.publish( msg );
}

// publish lidar data as point cloud
void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = "/map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);

}

// compute velocity and position of the bucket from current image 
object computeVelocities(vector<object>& objects, ros::Time& time, cv::Mat& image, cv::Mat& pred, const imageParams& param)
{
  double dt = ( time - pastTime ).toNSec() / pow( 10, 9 );
  object result;
  if( dt < 0.001 )
    {
      pastTime = time;
      return result;
    }
  
  for( int i = 0 ; i < objects.size() ; i++ )
    {
      for( int j = 0 ; j < past_objects.size() ; j++ )
	{
	  double dist = euclidean_distance( objects[i].center, past_objects[i].center );
	  if( dist < 50 && dist < objects[i].distDiff )
	    {
	      double dx = ( objects[i].center.x - past_objects[j].center.x ) / dt;
	      double dy = ( objects[i].center.y - past_objects[j].center.y ) / dt;
	      double vel = fabs( sqrt( pow( dx, 2) + pow( dy, 2 ) ) );
	      cv::Point2f bucketPos = getBucketPos( param, objects[i] );
	      //double distance = sqrt( pow( bucketPos.x, 2 ) + pow( bucketPos.y, 2 ) );
	      //double theta = atan2( bucketPos.y, bucketPos.x );
	      if( vel > 150.0 || vel < 1.0 || fabs(dx) < 0.01 || fabs(dy) < 0.01 )
		continue;

	      //objects[i].distance = distance;
	      //objects[i].theta = theta;
	      //objects[i].vel = ( distance - past_objects[j].distance ) / dt;
	      //objects[i].velTheta = ( theta - past_objects[j].theta ) / dt; 
	      objects[i].vx = dx;
	      objects[i].vy = dy;
	      objects[i].distDiff = dist;
	      found++;
	      if( firstExecution )
	      {
		KF.transitionMatrix = (Mat_<float>(4, 4) << 1.0f,0,dt,0,   0,1.0f,0,dt,  0,0,1.0f,0,  0,0,0,1.0f);
		initKF();
		KF.statePre.at<float>(0)=bucketPos.x;
		KF.statePre.at<float>(1)=bucketPos.y;
		KF.statePre.at<float>(2)=0.0;//dx / param.scale;// initial v_x
		KF.statePre.at<float>(3)=0.0;//dy / param.scale;//initial v_y
		
		firstExecution = false;
		corrected = true;
	      }
	      else
		{
		  //cv::Point2f bucketPos = getBucketPos( param, objects[i] );
		  correctKF( bucketPos, { 0, 0 } );
		}
	      result = objects[i];
	      break;
	    }
	}
    }
  pastTime = time;
  past_objects = objects;
}  

// detect the bucket from the image using canny and hough transform
void detectBin(cv::Mat& image, ros::Time& time, const imageParams& param)
{
  cv::Mat canny;
  std::vector<cv::Vec3f> circles;
  //cv::Canny(image, canny, 900, 1, 5);
  cv::Mat image_blur;
  GaussianBlur( image, image_blur, Size(7, 7), 2, 2 );

  cv::HoughCircles( image_blur, circles, CV_HOUGH_GRADIENT, 1.2, 900, 30, 1, 6, 8 );
  vector<object> objects;
  for( size_t i = 0; i < circles.size(); i++ ) 
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      object obj;
      obj.center = center;
      obj.radius = radius;
      objects.push_back( obj );
    }
  static cv::Mat pred;
  if( !firstExecution )
    {
      double dt = ( time - pastTime ).toNSec() / pow( 10, 9 );
      KF.transitionMatrix = (Mat_<float>(4, 4) << 1.0f,0,dt,0,   0,1.0f,0,dt,  0,0,1.0f,0,  0,0,0,1.0f);

      cv::Point prevCenter = { 0, 0 };
      if( !pred.empty() )
	prevCenter = { ( pred.at<float>(0) * param.scale ) + param.offset_x, ( pred.at<float>(1) * param.scale ) + param.offset_y };

      pred = KF.predict();

      cv::Point center = { ( pred.at<float>(0) * param.scale ) + param.offset_x, ( pred.at<float>(1) * param.scale ) + param.offset_y };
      cv::circle( image_blur, center, 7, Scalar(255), -1 );
      object obj;
      obj.center.x = center.x;
      obj.center.y = center.y;
      obj.vx = ( center.x - prevCenter.x ) / dt;
      obj.vy = ( center.y - prevCenter.y ) / dt;
      publish_odometry( obj, param );
    }

  object bucket;
  if( past_objects.size() == 0 )
    {
      pastTime = time;
      past_objects = objects;
    }
  else
    bucket = computeVelocities( objects, time, image_blur, pred, param );

  
  bitwise_or( image_blur, image, image );
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  if( publishImage )
    pub_image.publish( msg );

  if( displayImage )
    {
      imshow( " laserImage: ", image );
      waitKey( 1 );
    }
}

// convert the laser points from sensor_msgs LaserScan to pointCloud
void convertLaserToCloud(const sensor_msgs::LaserScan::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  static imageParams param;
  int sz = input->ranges.size();
  input_cloud->points.resize( sz );
  pcl::PointCloud<pcl::PointXYZ> temp_cloud;
  static pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud( new pcl::PointCloud<pcl::PointXYZ> );
  
  double angle_min = input->angle_min;
  double angle_max = input->angle_max;
  double angle_inc = input->angle_increment;
  int i = 0;
  bool firstPrint = true;
  pcl::PointXYZ minPt;
  pcl::PointXYZ maxPt;
  param.scale = 50.0;
  int j =0;
  int old_sz = old_cloud->size();
  for( double angle = angle_min ; angle < angle_max ; angle += angle_inc )
    {
      
      double x = input->ranges[i] * cos( angle );
      double y = input->ranges[i] * sin( angle );
      if( x > 50 || y > 50 )
	continue;
      pcl::PointXYZ p;
      p.x = x * param.scale;
      p.y = y * param.scale;
      p.z = 0.01;
      input_cloud->points[i] = p;
      if( old_sz > 0 )
	{
	  double dist = euclidean_distance( { p.x, p.y }, { old_cloud->points[i].x, old_cloud->points[i].y } );
	  if( fabs( dist ) > 1.0 )
	    temp_cloud.points.push_back( input_cloud->points[i] );
	}	
      i++;
    }
  old_cloud = input_cloud;
  param.offset_x = 200;//minPt.x * -1 * 1.1;
  param.offset_y = 200;//minPt.y  * -1 * 1.1;
   
  param.cols = 850;//maxPt.y - minPt.y;
  param.rows = 550;//maxPt.x - minPt.x;
  cv::Mat map( param.rows, param.cols, CV_8UC1, cv::Scalar( 0 ) );

  for( pcl::PointXYZ p : temp_cloud.points )
  {
    int x = param.offset_x + p.x;
    int y = param.offset_y + p.y;
    if( x >= param.rows || y >= param.cols || x < 0 || y < 0 )
      continue;
    map.at<uchar>( param.offset_x + p.x, param.offset_y + p.y ) = 255;
  }
  detectBin( map, input->header.stamp, param );
  publish_cloud( pub_cloud, input_cloud );
}

// Laser callback
void laser_cb(const sensor_msgs::LaserScan::Ptr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  convertLaserToCloud( input, input_cloud );
  cb++;
  corrected = false;
}


int main(int argc, char** argv)
{

  ros::init (argc,argv,"laser_object_tracker");
  ros::NodeHandle nh;
  nh.getParam("/laser_object_tracker/pub_image", publishImage);
  nh.getParam("/laser_object_tracker/display_image", displayImage);
  
  ros::Subscriber sub = nh.subscribe ("laser_horizontal_front", 1, laser_cb);
  image_transport::ImageTransport it(nh);
  pub_image = it.advertise("image", 1);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
  pub_bucket_data = nh.advertise<nav_msgs::Odometry> ("bucket_data", 1);
  ros::spin();
}
