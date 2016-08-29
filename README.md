# RIPVR-Controller

The project where my final electronics program grade would have inevitably died if I didn't complete it.

## Overview

![Final Product](http://i.imgur.com/5KWW1lE.jpg)

This repository includes assets and code for my high school senior project in which I designed and developed a controller for virtual reality from the ground up. The controller uses two infrared-lit spheres and the DK2 camera to track position of each controller, with two MPU-9250s for rotation tracking, two analog joysticks, 5 grip buttons and 4 general purpose buttons. It has only been tested using a DK2 on Linux. A demo application utilizing this controller can be found at [here](https://github.com/shinyquagsire23/OpenJK/tree/VR-Hands).

## Python Daemon

ripvr-daemon.py uses OpenCV2, v4l2, numpy, and other libraries to process image data from the DK2 camera to get position, rotation, and input data into a memmapped IPC buffer. The script also contains experimental code for stealing camera data from ovrd itself.
