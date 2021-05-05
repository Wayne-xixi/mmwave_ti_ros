Driver has been tested with Ubuntu 18.04 with ROS Melodic.

Dependencies installation:
- sudo apt install ros-melodic-radar-msgs

Installation:
- git clone https://github.com/juanmanuelfernandez-iertec/mmwave_ti_ros.git
- cd mmwave_ti_ros/ros_driver/
- catkin_make
- source devel/setup.bash

Launch:
- cd mmwave_ti_ros/ros_driver/
- source devel/setup.bash
- roslaunch ti_mmwave_rospkg 1843_multi_mrr_0.launch
- roslaunch ti_mmwave_rospkg 1843_multi_mrr_1.launch

Troubleshooting:
- Enable serial ports:
    - sudo chmod 666 /dev/ttyACM0
    - sudo chmod 666 /dev/ttyACM1
- Change the name of the used serial port:
    - Open mmwave_ti_ros/ros_driver/src/ti_mmwave_rospkg/launch/1843_multi_mrr_x.launch.
    - Change the serial port name in <param name="data_port" value="/dev/ttyACM0"  /> to the desired one.

Description:
- The original driver has been modified to support mmwave_automotive_toolbox_3_3_0\labs\lab0007_medium_range_radar.
- Radar perform measures in 2 modes:
    - Medium Range Radar (MRR): range of 120 m.
    - Ultra Short Range Radar (USRR): range of 30 m.
- The following topics are used by the driver to send data to another nodes:
    - /ti_mmwave/radar_detection_mrr_x and /ti_mmwave/radar_detection_usrr_x: Detected objects in MRR and USRR modes. Format (radar_msgs/RadarDetectionArray.msg):
        header:
          seq: uint32             # Always 0
          stamp:
            secs: uint32          # seconds since epoch
            nsecs: uint32         # nanoseconds since secs
          frame_id: string        # Frame ID used for multi-sensor scenarios
        detections[]:
          detection_id: uint16    # ID of the detection frame (Every frame starts with 0)
          position:               # Only the x and y components are valid.
            x: double             # Point x coordinates in m (front from antenna)
            y: double             # Point y coordinates in m (left/right from antenna, right positive)
            z: double             # Point z coordinates in m (up/down from antenna, up positive)
          velocity:               # Range rate rectangular transformation to x and y components
            x: double             # Radar measured range rate x coordinates in m/s
            y: double             # Radar measured range rate y coordinates in m/s
            z: double             # Always 0
          amplitude: float64      # Radar measured intensity in dB
    - /ti_mmwave/radar_track_mrr_x and /ti_mmwave/radar_track_usrr_x: Detected tracks in MRR and USRR modes. Format (radar_msgs/RadarTrackArray.msg):
        header:
          seq: uint32             # Always 0
          stamp:
            secs: uint32          # seconds since epoch
            nsecs: uint32         # nanoseconds since secs
          frame_id: string        # Frame ID used for multi-sensor scenarios
        tracks[]:
          track_id: uint16        # ID of the track frame (Every frame starts with 0)
          track_shape:
            points[]:             # Four points that represent a rectangle in the xy plane
              x: float32          # Point x coordinates in m (front from antenna)
              y: float32          # Point y coordinates in m (left/right from antenna, right positive)
              x: float32          # Always 0
          linear_velocity:        # Only the x and y components are valid. Only used by MRR
              x: float64          # Track measured velocity x coordinates in m/s
              y: float64          # Track measured velocity y coordinates in m/s
              x: float64          # Always 0
          linear_acceleration:    # Not used
              x: float64
              y: float64
              x: float64
- The following topics are used by the driver to send data to rviz:
    - /ti_mmwave/radar_detection_mrr_rviz_x and /ti_mmwave/radar_detection_usrr_rviz_x: Point cloud of detected objects in MRR and USRR modes.
    - /ti_mmwave/radar_track_mrr_rviz_x and /ti_mmwave/radar_track_usrr_rviz_x: Marker array of tracks in MRR and USRR modes.
