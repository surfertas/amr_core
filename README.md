ROS packages related to autonomous mobile robot (amr) powered by Raspberry Pi
---

Related Posts:
1. [Autonomous Mobile Robot #1: Data collection to a trained model](https://surfertas.github.io/amr/deeplearning/machinelearning/2018/03/31/amr-1.html)
2. [Autonomous Mobile Robot #2: Inference as a ROS service](https://surfertas.github.io/amr/deeplearning/machinelearning/2018/04/01/amr-2.html)
3. [Autonomous Mobile Robot #3: Pairing with a PS3 Controller for teleop](https://surfertas.github.io/amr/deeplearning/machinelearning/2018/12/22/amr-3.html)

Note: Readme is incomplete will work to detail out requirements overtime.


### Training
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
The setup will require a external SSD connected to the raspi via the specified device driver.

Launch files are found [here](https://github.com/surfertas/amr_core/tree/master/amr_worker/amr_bringup/launch).

On Raspi:
```
$ roslaunch amr_bringup amr_teleop_bringup.launch
```
* Teleop assumes PS3 dualshock3 controller
* Bluetooth setup [instructions](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle) for PS3 controller. This needs to be done once `amr_teleop_bringup.launch` is launched in order to control the robot wireless.

At this point, your robot should be subscribing to topics related to images, and
commands(throttle and steer). Confirm that such topics are being published
by inputting `rostopic list` on command line.

In the `data_storage.yaml` configuration file, you can set the frequency at
which the system saves the features (image) and label (commands) to disk. Note
that you need to specify where you want to store the data.

Once data has been collected you can retrieve the pickle file, and use the
training repo found in `amr_models` to train the appropriate model.

### Using the trained model

Once the model has been trained, place the saved model file in the models folder
of the appropriate package in `amr_master`. (e.g. place the controller model in
`/models` of `amr_nn_controller_service`. Make sure the path to the model file
is updated in `./config`.

On Jetson TX initiate the service by:
```
$ roslaunch amr_nn_controller_service.launch
```

On Raspi launch the neural network driven controller by:
```
$ roslaunch amr_bringup amr_nn_bringup.launch
```

The intended set up is that `amr_worker` is running on an edge machine, (e.g.
Raspberry Pi) while `amr_master` should be running on a more powerful master
resource (e.g. Jetson TX).

