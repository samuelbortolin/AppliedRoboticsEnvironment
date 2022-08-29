# Install OpenCV 3.3.1

Setup

```
$ sudo apt-get update
```

Install dependencies

```
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config # these are mandatory
$ sudo apt-get install -y qt5-default libvtk6-dev # gui
$ sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev # media I/O
$ sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev # video I/O
```

Install the library

```
$ wget https://github.com/opencv/opencv/archive/3.3.1.zip
$ unzip 3.3.1.zip
$ rm 3.3.1.zip
$ mv opencv-3.3.1 OpenCV
$ cd OpenCV
$ mkdir build
$ cd build
$ cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_EXAMPLES=ON ..
$ make -j4
```

Install OpenCV

```
$ sudo make install
$ sudo ldconfig
```

Check installation

```
$ pkg-config --cflags opencv # get the include path (-I)
$ pkg-config --libs opencv   # get the libraries path (-L) and the libraries (-l)
```

