#Download and Install SDF

## Ubuntu

    sudo apt-get install libsdformat6-dev

# Compiling From Source (Ubuntu)

## Prerequisites

Make sure you have removed the Ubuntu pre-compiled binaries before installing from source

    sudo apt-get remove 'libsdformat*' 'sdformat*'

If you have previously installed from source, be sure you are installing to the same path location or that you have removed the previous installation from source version manually.

## Install Required Dependencies

Install prerequisites.  A clean Ubuntu system will need:

### build tools, ruby for building xml schemas, tinyxml, and boost:

    sudo apt-get install ruby-dev build-essential libtinyxml-dev libboost-all-dev cmake mercurial pkg-config

### ignition-math4:

for Ubuntu distros earlier than bionic, the OSRF package repository should be used:

    sudo apt-get install lsb-release
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
    sudo apt-get update

for Ubuntu bionic and later, skip to here:

    sudo apt-get install libignition-math4-dev

## Build And Install SDFormat

This section describes how to install SDFormat into `/usr`,
which greatly simplifies the process of linking to and loading `libsdformat`
by downstream software.
This may be suitable for use inside a docker container,
but care should be taken when installing to `/usr` since installing the
debian packages would overwrite the version that you built from source,
and it can be very difficult to debug problems that arise from this system state.
It is often safer to install to a local folder and set the `LD_LIBRARY_PATH`
and other environment variables appropriately.
To simplify this type of installation, a
[catkin workspace](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#Installinacatkinworkspace)
or
[colcon workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
may be used along with the corresponding build tools that generate
shell scripts for setting the necessary environment variables.

### Installing into `/usr`

1. Clone the repository into a directory in your home folder:

        mkdir ~/sdf_source
        cd ~/sdf_source/
        git clone https://github.com/ignitionrobotics/sdformat

1. Change directory into the sdformat repository and switch to the sdf6 branch

        cd sdformat
        git checkout sdf6

   **Note: the <tt>default</tt> branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the sdf6 branch if you desire more stability**

1. Create a build directory and go there

        mkdir build
        cd build

1. Build and install

        cmake .. -DCMAKE_INSTALL_PREFIX=/usr
        make -j4
        # make install will work as the root user in a docker container
        # otherwise you may need to use `sudo make install`
        make install

## Uninstalling Source-based Install ##

If you need to uninstall SDF or switch back to a debian-based install of SDF when you currently have installed SDF from source, navigate to your source code directory's build folders and run make uninstall:

    cd ~/sdf_source/sdformat/build
    # make uninstall will work as the root user in a docker container
    # otherwise you may need to use `sudo make uninstall`
    make uninstall
