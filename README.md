# FERS - Flexible Extensible Radar Simulator

---

FERS is a simulator for simulating the performance and output of a variety of radar systems. It is designed to support a wide range of traditional and modern radar system designs. Note that FERS is currently under active development, and thus may contain bugs and incomplete features.

The features which are currently implemented are:

* Creation of returns from arbitrary pulse shapes
* Simple propagation, doppler and phase model
* Modelling of Multistatic and Monostatic radar systems
* Modelling of CW and Pulsed radars
* Export to CSV, XML and HDF5 file formats
* Proper range-gate and timing for pulsed radars
* Modelling of 1/f noise on local oscillators
* Effects of multipath propagation

FERS is written in standard C++ and should compile and run on many architectures and operating systems. Please report successes and failures of running this software on non-Linux and non-x86 platforms to the authors, so we can improve the software and make it more portable.

#### AUTHORS

FERS was initially written by Marc Brooker and published in 2008. Significant modifications have since been made by Craig Tong, Stephen Paine, Darryn Jordan, and Yaaseen Martin.

#### BUILDING FERS

FERS depends on a number of external libraries, which you need to install before attempting to build FERS. The libraries you need to have installed are:

* Boost (at least boost::threads, boost::system and boost::random header) which is available freely from http://www.boost.org
* FFTW3, which is available freely from http://www.fftw.org
* libhdf5
* TinyXML, which is available freely from http://sourceforge.net/projects/tinyxml/
* CMake, which is available freely from https://cmake.org/

On a Debian or Ubuntu system (or pretty much any other decent GNU/Linux distribution) these libraries should be available pre-packaged for your installing pleasure. These can be installed using the following commands in the Terminal:

```bash
sudo apt-get install libboost-all-dev libfftw3-dev libhdf5-dev libhdf5-serial-dev build-essential cmake cmake-qt-gui python-all-dev libtinyxml-dev
cd /usr/lib/x86_64-linux-gnu
sudo ln -sf libhdf5_serial.so libhdf5.so
sudo ln -sf libhdf5_serial_hl.so libhdf5_hl.so
```

A system reboot may be required after this. Then, to install FERS on a Linux-based system:

* Download and extract the FERS repository
* Navigate to the extracted directory (by default: `cd fers/`)
* Enter the following commands in the Terminal:

```bash
mkdir build
cd build
cmake ../
make
```

A "fers" binary will be then be placed in the "/fers/src" directory which can be copied to a location of your choice. This can then be accessed using the ```./fers <input.fersxml>``` command in the "/fers/src" folder.

FERS can also be built in debug mode by replacing the ```bashcmake ../``` with ```bashcmake -DCMAKE_BUILD_TYPE=Debug ../```. It is recommended that separate build folders be used for release and debug builds.

#### ACKNOWLEDGEMENTS

Thanks are extended to the authors of TinyXML, FFTW, CMake, and Boost for making their excellent software freely available.

#### COPYRIGHT NOTICE

FERS is covered by the following copyright notice. Should you wish to acquire a copy of FERS not covered by these terms, please contact the Department of Electrical Engineering at the University of Cape Town.

Please note that this copyright only covers the source code, program binaries and build system of FERS. Any input files you create and results created by the simulator are not covered by this notice and remain copyright of their original author. This copyright notice does not cover the source code of tinyxml (found in the "tinyxml" directory). Please see the file readme.txt in that directory for a copyright notice for that code. TinyXML can, however, be freely distributed along with the code of FERS.

FERS - The Flexible Extensible Radar Simulator
Copyright (C) 2006 Marc Brooker and Michael Inggs

This program is free software; you can redistribute it and/or modify it under the terms of version 2 of the GNU General Public License as published by the Free Software Foundation. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
