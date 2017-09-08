# Compiling on Debian

## Requirements

`sudo apt-get install autoconf intltool python-gobject-dev gobject-introspection gtk-doc-tools libgstreamer0.10-dev python-gst0.10-dev libxml2-dev libcfitsio-dev libboost-dev libopencv-dev python-opencv v4l-utils libglib2.0-dev libudev1 libssl-dev libusb-dev libaudit-dev libusb-1.0-0-dev libudev-dev libv4l-dev libgstreamer-plugins-base1.0-dev libgtk-3-devlibgtk-3-dev libnotify-dev gstreamer0.10-plugins-good gstreamer0.10-plugins-bad gstreamer0.10-plugins-good gstreamer0.10-plugins-bad gstreamer0.10-plugins-ugly gstreamer0.10-tools libghc-gtksourceview2-dev gobject-introspection gtk-doc-tools libgstreamer0.10-dev python-gst0.10-dev gobject-introspection gtk-doc-tools libgstreamer0.10-dev python-gst0.10-dev  gtk-3.0 gstreamer1.0-plugins-bad`



## Aravis Dependency
from version 5, Aravis supports USB3 cameras

get and compile:https://github.com/AravisProject/aravis

might need to run:
`gst-inspect-1.0 bayer2rgb`



## Compiling freeture

`git clone https://github.com/fripon/freeture.git`

`cd freeture`

`./configure ARV_CFLAGS='-I/usr/local/include/aravis-0.6/' ARV_LIBS='-L/usr/local/lib/ -laravis-0.5 -lm'`



## Point Grey/FLIR note on working with Aravis

https://www.ptgrey.com/tan/10849

