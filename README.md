# image_transport_ws

## Run Command

### Camera node activation
~~~
$ cd image_transport_ws
$ source devel/setup.bash
$ roslaunch my_libuvc_camera KYT_camera.launch
~~~

### Gray-Scale Image
~~~
$ cd image_transport_ws
$ source devel/setup.bash
$ rosrun image_transport_tutorial gray_scale image_input:=/camera/image_raw image_output:=gray_image
~~~

### Binarized Image
~~~
$ cd image_transport_ws
$ source devel/setup.bash
$ rosrun image_transport_tutorial binary image_input:=/gray_image image_output:=/bin
~~~
