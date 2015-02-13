# CannyBot
getting a robot to draw the shapes it "sees" using image processing

http://stackoverflow.com/a/14945782

# Refrences:
* http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
* http://www.fanjita.org/serendipity/archives/57-Capturing-webcam-video-with-OpenCV-on-Raspberry-Pi-Arch-Linux.html
* http://stackoverflow.com/questions/11987483/opencvs-canny-edge-detection-in-c

## Setting up OpenCV v2.4.10 in Visual Studio
### Installation
* Download OpenCV v2.4.10 from https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.10/opencv-2.4.10.exe/download.
* Unzip and put opencv folder in location C:\
* rename opencv folder to "opencv"

### Add System Environment Variable to PATH
C:\opencv\build\x86\vc12\bin

### C++
Additional Include Directories:
C:\opencv\build\include
C:\opencv\build\include\opencv
C:\opencv\build\include\opencv2

### Linker
Additional Library Directories:
C:\opencv\build\x86\vc12\lib

Additional Dependencies:
opencv_stitching2410d.lib
opencv_contrib2410d.lib
opencv_videostab2410d.lib
opencv_superres2410d.lib
opencv_nonfree2410d.lib
opencv_gpu2410d.lib
opencv_ocl2410d.lib
opencv_legacy2410d.lib
opencv_ts2410d.lib
opencv_calib3d2410d.lib
opencv_features2d2410d.lib
opencv_objdetect2410d.lib
opencv_highgui2410d.lib
opencv_video2410d.lib
opencv_photo2410d.lib
opencv_imgproc2410d.lib
opencv_flann2410d.lib
opencv_ml2410d.lib
opencv_core2410d.lib