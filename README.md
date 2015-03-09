# CannyBot
getting a robot to draw the shapes it "sees" using image processing

Currently only works with Windows (still testing on OSX)!

To run just type,
* python CannyBot.py

## Installation

On Windows:
* http://docs.opencv.org/trunk/doc/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html

On OSX:
* brew tap homebrew/science
* brew install opencv
* cd /usr/local/Cellar/opencv/2.x.x/
* cd /Library/Python/2.7/site-packages/
* ln -s /usr/local/Cellar/opencv/2.x.x/lib/python2.7/site-packages/cv.py cv.py
* ln -s /usr/local/Cellar/opencv/2.x.x/lib/python2.7/site-packages/cv2.so cv2.so
* Done!

On Ubuntu:
* sudo apt-get update
* sudo apt-get upgrade
* sudo apt-get dist-upgrade
* sudo apt-get install build-essential cmake
* sudo apt-get install python-dev python-tk python-numpy python3-dev python3-tk python3-numpy
* sudo apt-get install sudo apt-get install libopencv-dev
* Done!

## Refrences

- http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
- http://www.fanjita.org/serendipity/archives/57-Capturing-webcam-video-with-OpenCV-on-Raspberry-Pi-Arch-Linux.html
- http://stackoverflow.com/questions/11987483/opencvs-canny-edge-detection-in-c
- https://opencvproject.wordpress.com/projects-files/detection-shape/
- http://www.arachnoid.com/cpptutor/student3.html
