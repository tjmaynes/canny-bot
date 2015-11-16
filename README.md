# CannyBot
getting the NAO Robot to draw the shapes it "sees" using OpenCV

![screenshot](/debug/NAOVISION.png)

*Was unable to test on OS X or Linux. Only confirmed working on Windows.*

To run just enter into terminal,
* python CannyBot.py

## Dependencies
* Python 2.7.8 / Anaconda 2.7
* NAO Python SDK for Windows
* OpenCV 2.3.1 (only version that currently runs for NAO)

## OpenCV Setup

### Windows
* http://docs.opencv.org/trunk/doc/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html

### OS X

```
brew tap homebrew/science

brew install opencv

cd /usr/local/Cellar/opencv/2.x.x/

cd /Library/Python/2.7/site-packages/

ln -s /usr/local/Cellar/opencv/2.x.x/lib/python2.7/site-packages/cv.py cv.py

ln -s /usr/local/Cellar/opencv/2.x.x/lib/python2.7/site-packages/cv2.so cv2.so
```

### Ubuntu

```
sudo apt-get update

sudo apt-get upgrade

sudo apt-get dist-upgrade

sudo apt-get install build-essential cmake

sudo apt-get install python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

sudo apt-get install sudo apt-get install libopencv-dev
```
