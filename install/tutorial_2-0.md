#Download and Install SDF

## Ubuntu

~~~
sudo apt-get install libsdformat2-dev
~~~

# Compiling From Source (Ubuntu)

## Prerequisites

Make sure you have removed the Ubuntu pre-compiled binaries before installing from source

    sudo apt-get remove 'libsdformat*' 'sdformat*'

If you have previously installed from source, be sure you are installing to the same path location or that you have removed the previous installation from source version manually.

## Install Required Dependencies

Install prerequisites.  A clean Ubuntu system will need:

~~~
sudo apt-get install build-essential libtinyxml-dev libboost-all-dev cmake mercurial pkg-config
~~~

## Build And Install SDFormat

1. Clone the repository into a directory in your home folder:

        mkdir ~/sdf_source
        cd ~/sdf_source/
        hg clone https://bitbucket.org/osrf/sdformat

1. Change directory into the sdformat repository and switch to the 2.0 branch

        cd sdformat
        hg up sdf_2.0

   **Note: the <tt>default</tt> branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the 2.0 branch if you desire more stability**

1. Create a build directory and go there

        mkdir build
        cd build

1. Build and install

        cmake .. -DCMAKE_INSTALL_PREFIX=/usr
        make -j4
        sudo make install

## Uninstalling Source-based Install ##

If you need to uninstall SDF or switch back to a debian-based install of SDF when you currently have installed SDF from source, navigate to your source code directory's build folders and run make uninstall:

~~~
cd ~/sdf_source/sdformat/build
sudo make uninstall
~~~
