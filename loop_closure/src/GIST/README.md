# LibGIST #

A simple C++ Wrapper of Lear's GIST implementation using OpenCV. The original C version can be found at: [http://lear.inrialpes.fr/software](http://lear.inrialpes.fr/software) and the GitHub repo:
[https://github.com/bwhite/imfeat/tree/master/imfeat/_gist](https://github.com/bwhite/imfeat/tree/master/imfeat/_gist).

## What is GIST descriptor? ##

GIST is a global representation of a digital image, it can be used in image classification and clustering. Detail explanation can be found at: [http://people.csail.mit.edu/torralba/code/spatialenvelope/](http://people.csail.mit.edu/torralba/code/spatialenvelope/)

## Usage ##

To compile the source code, fftw3 is needed, which can be found at: [http://www.fftw.org/](http://www.fftw.org/), and OpenCV: [http://www.opencv.org](http://www.opencv.org). To use the wrapper class, see the example provided in main.cpp.