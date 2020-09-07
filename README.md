# laser-object-tracker
Ros node to track a bucket of radius 280mm detected using a stationary 2D lidar sensor

**Algorithm.**
- Construct a point cloud from a laser scan.
- Project the points on to an image at a larger scale (50) in this project as image pixels are ints not double
- Detect edges using canny edge detector and use HoughCircles from openCV to detect circles of radius between 6 to 8 pixels ( 0.24m to 0.32m radius )
- Track the circle center by predicting the position using kalman filter from openCV.

**Dependencies:**
- ROS1
- OpenCV
- PCL

**Topics:**
  - Subscribes to:
    - /laser_horizontal_front
  - Publishes to:
    - /image 
    - /bucket_data (units meters)


**Steps to execute:**
- Download and install the dependencies ROS1, OpenCV, PCL.
- Clone the repository into the catkin_ws/src directory using git clone repo.git
- Change the params in the laser_object_tracking.launch to display or publish image as needed
- Run the launch file using roslaunch laser_object_tracking laser_object_tracking.launch .
- If there is a permission error then provide permission to the laser_object_tracker node in catkin_ws/devel/lib/laser_object_tracking by using "chmod +x laser_obeject_tracker"
- play the bag file to publish laser scan
- The image can be visualized on rviz by selecting /image topic "raw" from rviz topic list if publish image in params of launch file is enabled 
- The position of the bucket is published on bucket_data

