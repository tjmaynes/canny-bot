# CannyBot
getting a robot to draw the shapes it "sees" using image processing

Currently only works with Windows (still testing on OSX)!

## Python Setup

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

## C++ Setup in Visual Studio

## Installation
* Download OpenCV v2.4.10 from [here]( https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.10/opencv-2.4.10.exe/download).
* Unzip and put opencv folder in location C:\{opencv}
* rename opencv folder to "opencv"

## Add System Environment Variable to PATH
* C:\opencv\build\x86\vc12\bin

## C++
Additional Include Directories:
* C:\opencv\build\include
* C:\opencv\build\include\opencv
* C:\opencv\build\include\opencv2

## Linker
Additional Library Directories:
* C:\opencv\build\x86\vc12\lib

Additional Dependencies (copy and paste):
* opencv_stitching2410d.lib
* opencv_contrib2410d.lib
* opencv_videostab2410d.lib
* opencv_superres2410d.lib
* opencv_nonfree2410d.lib
* opencv_gpu2410d.lib
* opencv_ocl2410d.lib
* opencv_legacy2410d.lib
* opencv_ts2410d.lib
* opencv_calib3d2410d.lib
* opencv_features2d2410d.lib
* opencv_objdetect2410d.lib
* opencv_highgui2410d.lib
* opencv_video2410d.lib
* opencv_photo2410d.lib
* opencv_imgproc2410d.lib
* opencv_flann2410d.lib
* opencv_ml2410d.lib
* opencv_core2410d.lib

## Refrences

- http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
- http://www.fanjita.org/serendipity/archives/57-Capturing-webcam-video-with-OpenCV-on-Raspberry-Pi-Arch-Linux.html
- http://stackoverflow.com/questions/11987483/opencvs-canny-edge-detection-in-c
- https://opencvproject.wordpress.com/projects-files/detection-shape/
- http://www.arachnoid.com/cpptutor/student3.html
