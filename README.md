### FreeTure [![Build Status](https://travis-ci.org/fripon/freeture.svg?branch=master)](https://travis-ci.org/fripon/freeture)[![Coverity Scan Build Status](https://scan.coverity.com/projects/6030/badge.svg)](https://scan.coverity.com/projects/6030 "Coverity Badge")
--------

*A Free software to capTure meteors*

`FreeTure` is a free open source (GPL license) meteor detection software used to monitor the sky with all-sky cameras to detect and record meteors and fireballs.

It is portable and cross-platform (Linux, Windows)

The project home page is http://fripon.org

Features
--------

- Supports GigE cameras (Tested with Basler acA1300-30gm and DMK23G445)
- Supports USB 2.0 cameras (Tested with stk1160 and Pinnacle Dazzle DVC 100 video grabber, DMx 31AU03.AS)
- Supports USB 3.0 cameras through Aravis 0.5+ (Tested with Point Grey/FLIR BFLY-U3-23S6M-C and Aravis 0.6)
- Internal computation of sun ephemeris
- Night and daytime (experimental) meteor detection modes
- Fits format in output https://en.wikipedia.org/wiki/FITS
- Possibility to run regular or scheduled long exposure acquisition
- Possibility to stack frames in order to keep a kind of history
- Supports frame cropping aka region of interest (Tested with Point Grey/FLIR BFLY-U3-23S6M-C)

Examples
--------

![grez_armainvilliers_20141229](https://raw.githubusercontent.com/fripon/freeture/master/data/gretz_armainvilliers-fireball-20141229.jpg)

![wien_20150905](https://raw.githubusercontent.com/fripon/freeture/master/data/wien-fireball-20150905.jpg)
