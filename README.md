SPQR Code Release based on the B-Human 2018 Code Release.



[SPQR Team website](http://spqr.diag.uniroma1.it/)



## Installation

### Install needed libraries
Open a terminal `Ctrl`+`Alt`+`t` (usually) and type the followings commands: <br>
* `$ sudo apt install build-essential cmake clang make qtbase5-dev libqt5opengl5-dev libqt5svg5-dev libglew-dev net-tools graphviz xterm qt5-default libvtk6-dev zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev libtbb-dev libpng16-16:i386 libtbb2:i386 libjpeg62:i386 libtiff5:i386 libv4l-0:i386` 

### Install the naoqi basic libraries
* move to the `Install/` folder - `$ cd RoboCup/spqrnao2019/Install`
* install the libraries - `./installAlcommon naoqi-sdk-2.8.5.10-linux64.tar.gz` ([lib](https://drive.google.com/file/d/100q36f5vNdozTamF16mB6niEqSJoGC9U/view?usp=sharing))

### Install OpenCV3.1
* dowload source of OpenCV- `$ wget https://github.com/opencv/opencv/archive/3.1.0.zip`
* unzip the files- `$ unzip 3.1.0.zip`
* move into OpenCV directory- `$ cd opencv-3.1.0`
* create the build directory- `$ mkdir build && cd build`
* generate the make file- `$ cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_CUDA=OFF ..` **if you have trouble with stdlib add this flag** `-DENABLE_PRECOMPILED_HEADERS=OFF`
* compile- `$ make -j#num of core of your cpu`
* install OpenCV- `$ sudo make install` and then- `$ sudo ldconfig`
* to use simrobot you have to change in the file **Config/settings.cfg** the parameter `scenario = ballbhuman`

### Compile the code 
* Only if you are using Ubuntu_16 you need to set Clang-6.0 : `$ sudo apt-get install -y clang-6.0`
* `$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 1000 `
* `$ sudo update-alternatives --install /usr/bin/clang++ clang /usr/bin/clang-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 1000 `
* `$ sudo update-alternatives --config clang `
* `$ sudo update-alternatives --config clang++ `
* move to `Linux/` folder - `$ cd RoboCup/spqrnao2019/Make/Linux`
* compile the code - `$ make CONFIG=<Develop/Debug/Release>` 

### Run SimRobot
* move to `Develop/` folder - `$ cd RoboCup/spqrnao2019/Build/Linux/SimRobot/<Develop/Debug/Release>/`
* run the exec - `$ ./SimRobot` 
