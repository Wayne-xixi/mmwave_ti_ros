Driver has been tested with Ubuntu 18.04 with ROS Melodic.

Installation:
- git clone https://github.com/juanmanuelfernandez-iertec/mmwave_ti_ros.git
- cd mmwave_ti_ros/ros_driver/
- catkin_make
- source devel/setup.bash

Launch:
- cd mmwave_ti_ros/ros_driver/
- source devel/setup.bash
- roslaunch ti_mmwave_rospkg 1843_mrr.launch

Troubleshooting:
- Enable serial ports:
	- sudo chmod 666 /dev/ttyACM0
	- sudo chmod 666 /dev/ttyACM1
- Change the name of the used serial port:
	- Open mmwave_ti_ros/ros_driver/src/ti_mmwave_rospkg/launch/1843_mrr.launch.
	- Change the serial port name in <param name="data_port" value="/dev/ttyACM0"  /> to the desired one.

Description:
- The original driver has been modified to support mmwave_automotive_toolbox_3_3_0\labs\lab0007_medium_range_radar.
- Radar perform measures in 2 modes:
	- Medium Range Radar (MRR): range of 120 m.
	- Ultra Short Range Radar (USRR): range of 30 m.
- The following topics are used by the driver to send data to another nodes:
	- /ti_mmwave/radar_object_mrr and /ti_mmwave/radar_object_usrr: Detected objects in MRR and USRR modes. Format:
		header: 
		  seq: 6264
		  stamp: 
			secs: 1538888235
			nsecs: 712113897
		  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
		point_id: 17              # Point ID of the detecting frame (Every frame starts with 0)
		x: 8.650390625            # Point x coordinates in m (front from antenna)
		y: 6.92578125             # Point y coordinates in m (left/right from antenna, right positive)
		z: 0.0                    # Point z coordinates in m (up/down from antenna, up positive)
		range: 11.067276001       # Radar measured range in m
		velocity: 0.0             # Radar measured range rate in m/s
		doppler_bin: 8            # Doppler bin location of the point (total bins = num of chirps)
		bearing: 38.6818885803    # Radar measured angle in degrees (right positive)
		intensity: 13.6172780991  # Radar measured intensity in dB
	- /ti_mmwave/radar_cluster_mrr and /ti_mmwave/radar_cluster_usrr: Detected clusters in MRR and USRR modes. Format:
		header: 
		  seq: 6264
		  stamp: 
			secs: 1538888235
			nsecs: 712113897
		  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
		x: 8.650390625            # Cluster center x coordinates in m (front from antenna)
		y: 6.92578125             # Cluster center y coordinates in m (left/right from antenna, right positive)
		x_size: 8.650390625       # Cluster size x coordinates in m
		y_size: 6.92578125        # Cluster size y coordinates in m
	- /ti_mmwave/radar_track_mrr and /ti_mmwave/radar_track_usrr: Detected tracked objects in MRR and USRR modes. Format:
		header: 
		  seq: 6264
		  stamp: 
			secs: 1538888235
			nsecs: 712113897
		  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
		x: 8.650390625            # Tracking center x coordinates in m (front from antenna)
		y: 6.92578125             # Tracking center y coordinates in m (left/right from antenna, right positive)
		x_vel: 8.650390625        # Tracking velocity x coordinates in m/s
		y_vel: 6.92578125         # Tracking velocity y coordinates in m/s
		x_size: 8.650390625       # Tracking size x coordinates in m
		y_size: 6.92578125        # Tracking size y coordinates in m
- The following topics are used by the driver to send data to rviz:
	- /ti_mmwave/radar_object_mrr and /ti_mmwave/radar_object_usrr: Point cloud of detected objects in MRR and USRR modes.
	- /ti_mmwave/radar_cluster_mrr and /ti_mmwave/radar_cluster_usrr: Marker array of detected clusters in MRR and USRR modes.
	- /ti_mmwave/radar_track_mrr and /ti_mmwave/radar_track_usrr: Marker array of detected tracked objects in MRR and USRR modes.
