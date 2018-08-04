Object Detection using Yolov2
---

The ros package calls a service `object_detection_service` to run object detection on
images published to `/webcam/image_raw` from a different machine.

The raw images were taken on using a Raspi Camera Module attached to a Raspberry
Pi 3, running on Ubuntu Mate 16.04.1 and ROS Kinetic. 

Inference was done on a Jetson TX1 running on Ubuntu 16.04.1, OpenCV 3.3.1 with
Cuda 9.0 enabled, ROS Kinetic, and PyTorch 0.5.0.

Inference is done using a pretrained model:
```
$ wget http://pjreddie.com/media/files/yolo.weights 
```
The original PyTorch code for inference was taken from
[here](https://github.com/marvis/pytorch-yolo2) and modified for use in this
custom ROS package and located in `/scripts`.

In order to run:
1. Update location of relevant files by modifying the paths in
`/config/models/yolo2.yaml`
1. Place `yolo.weights` downloaded from pjreddie.com, in the `/data` directory.
1. Start publishing images to `/webcam/image_raw`.
1. Go to `/launch` and run `roslaunch amr_object_detection.launch`

