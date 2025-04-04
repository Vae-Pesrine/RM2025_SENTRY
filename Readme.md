# SENTRY SIMULATION FOR ROBOMASTER 

## 一.Run

run the simulation environment
```SHELL
roslaunch sentry_simulation view.launch
```

run the localization algorithm
```SHELL
roslaunch fast_lio mapping_mid360.launch
roslaunch sentry_localization sentry_localization.launch
```

run the navigation (you can use the fast_planner_nmpc or use the move base package)
```SHELL
roslaunch plan_manage 16_lidar.launch
roslaunch plan_manage rviz.launch
rosrun mpc_tracking mpc_tracking_node
```

## 二.Change the version of pcl to build the small_gicp_localization

I choose to compile from source code to change the version of pcl. 

New version->[pcl-1.12.0](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.12.0)   

```SHELL
cd ~/DownLoads
unzip pcl-1.12.0.zip
sudo mv pcl-1.12.0 /usr/local/include
mkdir build && cd build
cmake .. 
make -j4   
sudo make install
```
The building process is running slow. Code "make -j4" means building with four threads, if your computer cannot afford it, you can run "make".  

```SHELL
cd /opt/ros/noetic/share/pcl_ros/cmake
sudo chmod 777 pcl_rosConfig.cmake  
```
Find the code below and change the pcl path to newest.If you follow the steps I mentioned above, you should change "/usr/include/pcl-1.10" to "/usr/local/include/pcl-1.12".

```SHELL
if(NOT "include;/usr/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu " STREQUAL " ")
  set(pcl_ros_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu")
 ```
```SHELL
cd /opt/ros/noetic/share/pcl_conversions/cmake
sudo chmod 777 pcl_conversionsConfig.cmake  
```
Just like "pcl_ros".

If you meet with the warning like "/usr/bin/ld: warning: libpcl_filters.so.1.10, needed by /usr/lib/x86_64-linux-gnu/libpcl_registration.so, may conflict with libpcl_filters.so.1.12", run code below.
```SHELL
sudo ln -sf /usr/local/lib/libpcl_filters.so.1.12 /usr/lib/x86_64-linux-gnu/libpcl_filters.so 
```

## 三.Install Ipopt and Casadi 
1.install some dependencies

```SHELL
sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev 
```

```SHELL
mkdir ~/Ipopt_pkg
cd Ipopt_pkg
```

2.install ASL

```SHELL
git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
cd ThirdParty-ASL
sudo ./get.ASL
sudo ./configure
sudo make
sudo make install
cd ..
```

3.install HSL

Download the [coinhsl](https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip)

```SHELL
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ~/Downloads
unzip coinhsl.zip ~/Ipopt_pkg/ThirdParty-HSL
cd ~/Ipopt_pkg/ThirdParty-HSL
sudo ./configure
sudo make
sudo make install
cd ..
```

4.install MUMPS

```SHELL
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
sudo ./get.Mumps
sudo ./configure
sudo make
sudo make install
cd ..
```

5.install Ipopt

```SHELL
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt
mkdir build
cd build
sudo ../configure
sudo make
sudo make test
sudo make install
```

6.prefect the environment

```SHELL
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
```
The Ipopt is installed successfully until now.

7.install the casadi use the bash
```SHELL
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


```SHELL
cd ~/
touch install-casadi.sh
sudo chmod +x install-casadi.sh
./install-casadi.sh
```
The Casadi is installed successfully until now.


