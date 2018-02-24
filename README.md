ROS packages related to autonomous mobile robot (amr) powered by Raspberry Pi
---

Note: Readme is incomplete will work to detail out requirements overtime.

1. Clone amr_core repository. 
```
$ git clone https://github.com/surfertas/amr_core.git
```
2. Install joy package (used for teleoperation)
```
$ sudo apt-get install ros-indigo-joy
```
3. Install `video_stream_opencv` package. 
```
$ git clone https://github.com/ros-drivers/video_stream_opencv.git
```
4. Configure `data_storage.yaml` found [here](https://github.com/surfertas/amr_core/blob/master/amr_data_processor/config/data_storage.yaml)

Launch files are found [here](https://github.com/surfertas/amr_core/tree/master/amr_bringup/launch).

Example launch:
```
$ roslaunch amr_teleop_bringup.launch
```
* Teleop assumes ps3 dualshock3 controller
