# <center>***SENTRY NAVIGATION FOR ROBOMASTER***<center> 

## TODO
 - [ ] fix the bug of ground segmentation when running on the center terrain
 - [ ] fix the bug of free space recovery (tf warning of repeated data when processing the recovery plugin)  
 - [ ] add the scan context helping relocalization when the odometry drifts(waiting for optimization)
 - [ ] add the stc

## **1.Framework**
```plaintext
src
│
├── sentry_bringup                
│
├── sentry_driver           
│   ├── livox_ros_driver2
│   └── serial_driver
│
├── sentry_localization           
│   ├── fast_lio        
│   ├── point_lio(version: gridmap)   
│   └── small_gicp_registration
│
├── sentry_navigation
│   ├── mbf_nav 
│   └── move_freespace_recovery
│
├── sentry_perception
│   ├── terrain_analysis
│   ├── pcd2gridmap
│   ├── ground_segmentation
│   └── livox_msgs_convert
│
└── sentry_simulation
    ├── sentry_description
    └── livox_laser_simulation
```


## **2.Dependencies**

- **1.[Small Gicp](https://github.com/koide3/small_gicp.git)**
  ```shell
  git clone https://github.com/koide3/small_gicp.git
  sudo apt-get install libeigen3-dev libomp-dev
  cd small_gicp
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```
- **2.[Teaser-Plusplus](https://github.com/MIT-SPARK/TEASER-plusplus.git)**
  ```shell
  git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
  cd TEASER-plusplus && mkdir build && cd build
  cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
  sudo make install
  sudo ldconfig
  ```
- **3.[Gtsam](https://github.com/borglab/gtsam)>=4.1.1**
  ```shell
  wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
  unzip gtsam.zip
  cd gtsam-4.1.1/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
  sudo make install   
  ```
- **4.Tbb(used for faster Quatro)**
  ```shell
  sudo apt install libtbb-dev
  ```
- **4.[3Dbbs](https://github.com/KOKIAOKI/3d_bbs)**
  ```shell
  # Note: If you are using Eigen3 below 3.4.0, git clone with --recursive
  # check the version of eigen on your device: pkg-config --modversion eigen3
  git clone https://github.com/KOKIAOKI/3d_bbs.git
  cd 3d_bbs
  mkdir build && cd build

  # cpu&&gpu version
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j
  sudo make install

  # cpu version only
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
  make -j
  sudo make install
  ```
  **NOTE: You may encounter certain warnings during the build process of both the CPU and GPU versions; however, these warnings are not meaningful. If you wish to address them, you can replace the original CMake commands for CUDA with the following commands in the CMakeLists.txt file.**
  ```shell
  ## find CUDA
  option(BUILD_CUDA "Build GPU ver" ON)
  if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  # Add the --expt-relaxed-constexpr flag to CUDA_NVCC_FLAGS
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --expt-relaxed-constexpr")
  # Suppress specific warnings
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -diag-suppress=20208")
  endif()
  ```

## **5.How to build**
+ use **catkin build** to build the package one by one or use less than four threads
  ```shell
  # nano gicp, quatro first
  catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
  catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
  catkin build -DCMAKE_BUILD_TYPE=Release
  source ./devel/setup.bash
  ```

## **6.How to run**

- run the simulation environment
  ```shell
  roslaunch sentry_brignup 01simu.launch
  ```

- run the localization algorithm
  ```shell
  roslaunch sentry_bringup 02localize.launch
  ```

- run the navigation algorithm
  ```shell
  roslaunch sentry_bringup 03nav.launch
  ```


## **Change the version of pcl to build the small_gicp_localization**

- 1.This small gicp library uses C++17 features. The pcl interface is not compatible with pcl older than 1.11 that uses boost::shared_ptr. And the default version of pcl is 1.10 when install ROS. So we install [pcl-1.12.0](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.12.0).

  ```shell
  wget -O pcl-1.12.0.zip https://github.com/PointCloudLibrary/pcl/archive/pcl-1.12.0.zip
  unzip pcl-1.12.0.zip
  sudo mv pcl-pcl-1.12.0 pcl-1.12.0
  sudo mv pcl-1.12.0 /usr/local/include
  cd /usr/local/include
  sudo mkdir build && cd build
  sudo cmake .. 
  sudo make -j4   
  sudo make install

  cd /opt/ros/noetic/share/pcl_ros/cmake
  sudo chmod 777 pcl_rosConfig.cmake 
  ```
  The building process is running slow. Code "make -j4" means building with four threads, if your computer cannot afford it, you can run "make". If you can't run the command ,try "sudo *".   

- 2.Change the default path of pcl

  ```shell
  cd /opt/ros/noetic/share/pcl_ros/cmake
  sudo chmod 777 pcl_rosConfig.cmake  
  ```

  Find the code below and change the pcl path to newest.If you follow the steps I mentioned above, you should change "/usr/include/pcl-1.10" to "/usr/local/include/pcl-1.12".
  ```shell
  if(NOT "include;/usr/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu " STREQUAL " ")
    set(pcl_ros_INCLUDE_DIRS "")
    set(_include_dirs "include;/usr/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu")
  ```

  Just like above.
  ```shell
  cd /opt/ros/noetic/share/pcl_conversions/cmake
  sudo chmod 777 pcl_conversionsConfig.cmake  
  ```
  
- **Note**: If you meet with the warning like "/usr/ bin/ld: warning: libpcl_filters.so.1.10, needed by /usr/lib/x86_64-linux-gnu/libpcl_registration.so, may conflict with libpcl_filters.so.1.12", run code below.The same goes for other pcl libraries. 
  ```shell
  sudo ln -sf /usr/local/lib/libpcl_filters.so.1.12 /usr/lib/x86_64-linux-gnu/libpcl_filters.so 
  ```

## Install Ipopt and Casadi for mpc solve(it's needless if you use move_base)
- 1.install some dependencies

  ```shell
  sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev 
  ```

  ```shell
  mkdir ~/Ipopt_pkg
  cd Ipopt_pkg
  ```

- 2.install ASL

  ```shell
  git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
  cd ThirdParty-ASL
  sudo ./get.ASL
  sudo ./configure
  sudo make
  sudo make install
  cd ..
  ```
- 3.install HSL

  Download the [coinhsl](https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip)

  ```shell
  git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
  cd ~/Downloads
  unzip coinhsl.zip ~/Ipopt_pkg/ThirdParty-HSL
  cd ~/Ipopt_pkg/ThirdParty-HSL
  sudo ./configure
  sudo make
  sudo make install
  cd ..
  ```

- 4.install MUMPS

  ```shell
  git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
  cd ThirdParty-Mumps
  sudo ./get.Mumps
  sudo ./configure
  sudo make
  sudo make install
  cd ..
  ```
  
- 5.install Ipopt

  ```shell
  git clone https://github.com/coin-or/Ipopt.git
  cd Ipopt
  mkdir build
  cd build
  sudo ../configure
  sudo make
  sudo make test
  sudo make install
  ```

- 6.prefect the environment

  ```shell
  cd /usr/local/include
  sudo cp coin-or coin -r
  sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
  sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
  sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
  ```
  The Ipopt is installed successfully until now.

- 7.install the casadi, run bash below
  ```shell
     #!/usr/bin/env bash
  
     set -e
  
     DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
     echo -e "\033[40;32m${DIR} \033[0m"
  
     wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-3.5.5-1.tar.gz
     tar -zxvf casadi-3.5.5-1.tar.gz
     echo -e "\033[40;32mdownload finish \033[0m"
  
     cd casadi-3.5.5.1
     mkdir build && cd build
     cmake .. -DWITH_IPOPT=ON -DWITH_EXAMPLES=OFF
     make -j4
     sudo make install
     sudo ldconfig
  
     sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
     sudo rm -fr casadi-3.5.5-1.tar.gz casadi-3.5.5.1
  ```
  The Casadi is installed successfully until now.


