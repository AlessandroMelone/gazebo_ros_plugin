# gazebo_ros_plugin
Thanks to Robert Krug repository: "Urdf & Gazebo files for a KUKA LBR iiwa R820 with electrical media flange" avaiable at https://github.com/rtkg/lbr_iiwa 


Implementation of a Gazebo plugin using ROS and KDL libraries that allow to decide a point in the cartesian space that the end-effector had to reach.

# How to use:

- copy the content of the repository in your ROS src folder

- $ ~/your_ros_ws catkin_make
  $ roslaunch lbr_iiwa_description gazebo_ctrl.launch
  click start simulation button on gazebo
  wait that the ROS_INFO message appear on the linux bash: "Ready to control!"
  
- open another bash shell to send topic message that plugin will read to know your desired end-effector position,
  you have two option to specify the position:
  
  1) Absolute, i.e. specify the point coordinates respect to the base frame. 
     In that case you have to write on the /gazebo_plugin/des_position topic. 
     For example:
  	
  	$rostopic pub -1 /gazebo_plugin/des_position geometry_msgs/Point -- '0.5' '-0.5' '0.6'
  
  2) Relative, i.e. specify the point coordinates respect to the actual end-effector position.
     In that case you have to write on the /gazebo_plugin/des_position topic. 
     For example:
  	
  	$rostopic pub -1 /gazebo_plugin/des_position_relative geometry_msgs/Point -- '-0.5' '0.2' '0.1'
  	
- after publishing the message the robot will start to move to reach the desired position
