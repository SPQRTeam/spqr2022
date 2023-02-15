
Code Release based on the B-Human 2018 Code Release.



[SPQR Team website](http://spqr.diag.uniroma1.it/)



## Installation

### Supported distros:
* Ubuntu 17.04/17.10/18.04/18.10/19.04/19.10/20.04/20.10
* Linux Mint 18.x / 20.x


### Install needed libraries
Open a terminal `Ctrl`+`Alt`+`t` (usually) and type the followings commands: <br>
* `$ sudo apt install build-essential cmake clang make qtbase5-dev libqt5opengl5-dev libqt5svg5-dev libglew-dev net-tools graphviz xterm qt5-default libvtk6-dev zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev libtbb-dev libpng16-16:i386 libtbb2:i386 libjpeg62:i386 libtiff5:i386 libv4l-0:i386` 

### Install the naoqi basic libraries
* move to the `Install/` folder - `$ cd RoboCup/spqr2022/Install`
* install the libraries - `./installAlcommon naoqi-sdk-2.8.5.10-linux64.tar.gz` 

### Install OpenCV3.1
* dowload source of OpenCV- `$ wget https://github.com/opencv/opencv/archive/3.1.0.zip`
* unzip the files- `$ unzip 3.1.0.zip`
* move into OpenCV directory- `$ cd opencv-3.1.0`
* create the build directory- `$ mkdir build && cd build`
* generate the make file- `$ cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_CUDA=OFF ..` **if you have trouble with stdlib add this flag** `-DENABLE_PRECOMPILED_HEADERS=OFF`
* compile- `$ make -j#num of core of your cpu`
* install OpenCV- `$ sudo make install` and then- `$ sudo ldconfig`

#### Possible problems at this step

The following errors may happen for some users, but not others. *If you incur into one, please report it to us as soon as possible, so we can update this wiki with the proper error message.*

* **IMPORTANT FOR ANACONDA/MINICONDA USERS:** Either of them can mess with the installation process by generating seemingly unrelated errors *(e.g. an undefined reference to libtiff)*. The easiest way is to temporarily disable them by following this procedure: [(source)](https://github.com/colmap/colmap/issues/188#issuecomment-440665679)
  - Open your .bashrc: `$ nano ~/.bashrc`
  - Locate the line that adds <ana/mini>conda to your PATH. It may look like `export PATH="/home/<user>/anaconda3/bin:$PATH` for anaconda, and similarly for miniconda. If you can't find it in .bashrc, try your .profile as well.
  - Comment out that line *(add a `#` as the first character of that line)*.
  - *Open a new terminal window* and carry out the OpenCV installation procedure as above.
  - Un-comment that line in your .bashrc
* **If you get an error message related to ffmpeg:** Install ffmpeg with the command `sudo apt install ffmpeg`
* **If the compiler says some flag (one of which may be CODEC_FLAG_GLOBAL_HEADER) was not declared:** You may have to add the missing defines to the OpenCV code. Open the file `<opencv folder>/modules/videoio/src/cap_ffmpeg_impl.hpp` and add the folowing lines: [(source)](https://stackoverflow.com/a/47005401)
```c++
    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020
```
* **If you get errors related to libasound:** Install the corresponding headers with the command `sudo apt install libasound2-dev`

### Compile the code 
**If you are using Ubuntu_16 or Mint_18,** you need to set Clang-6.0 before you compile: 
* `$ sudo apt-get install -y clang-6.0 `
* `$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 1000 `
* `$ sudo update-alternatives --install /usr/bin/clang++ clang /usr/bin/clang-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.8 100 `
* `$ sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 1000 `
* `$ sudo update-alternatives --config clang `
* `$ sudo update-alternatives --config clang++ `

To compile **(for all OS versions)**:
* move to `Linux/` folder - `$ cd RoboCup/HomeworkMAS-RoboCup/Make/Linux`
* compile the code - `$ make CONFIG=<Develop/Debug/Release>` 
