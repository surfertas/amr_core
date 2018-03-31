ROS packages related to autonomous mobile robot (amr) powered by Raspberry Pi
---

Related Posts:
1. [Autonomous Mobile Robot #1: Data collection to a trained model](https://surfertas.github.io/amr/deeplearning/machinelearning/2018/03/31/amr-1.html)


Note: Readme is incomplete will work to detail out requirements overtime.

1. Clone amr_core repository. 
```
$ git clone https://github.com/surfertas/amr_core.git
```
2. Install joy package (used for teleoperation) into `amr_worker`.
```
$ sudo apt-get install ros-indigo-joy
```
3. Install `video_stream_opencv` package into `amr_worker`.
```
$ git clone https://github.com/ros-drivers/video_stream_opencv.git
```
4. Configure `data_storage.yaml` found [here](https://github.com/surfertas/amr_core/tree/master/amr_worker/amr_data_processor/config)

Launch files are found [here](https://github.com/surfertas/amr_core/tree/master/amr_worker/amr_bringup/launch).

The intended set up is that `amr_worker` is running on an edge machine, (e.g.
Raspberry Pi) while `amr_master` should be running on a more powerful master
resource (e.g. Jetson TX).

On Raspi:
```
$ roslaunch amr_bringup amr_teleop_bringup.launch
```

On Jetson TX1 launch the service hub which launch all services associated with
the master. (e.g. object detection)
```
$ roslaunch amr_service_hub amr_service_hub.launch
```

* Teleop assumes ps3 dualshock3 controller
