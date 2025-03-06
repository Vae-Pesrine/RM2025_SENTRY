


# SENTRY SIMULATION FOR RM

## run

启动仿真环境
```SHELL
roslaunch sentry_simulation view.launch
```

启动定位算法:里程计和重定位算法
```SHELL
roslaunch fast_lio mapping_mid360.launch
roslaunch sentry_localization sentry_localization.launch
```

启动导航部分
```SHELL
roslaunch plan_manage 16_lidar.launch
roslaunch plan_manage rviz.launch
rosrun mpc_tracking mpc_tracking_node
```


### update the version of pcl to build the small_gicp_localization

download [pcl-1.12.0](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.12.0)  

```SHELL
cd ~/DownLoads
unzip pcl-1.12.0.zip
mv pcl-1.12.0 /usr/include
mkdir build && cd build
cmake .. && make -j4   
sudo make install
```
```SHELL
cd /usr/local/include/share 
```

find pcl_ros and pcl_conversions packages, change the pcl path to the newest, we can build the package finally


### How to install Ipopt and Casadi 


1. install some dependencies

```SHELL
sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev 
```

```SHELL
mkdir ~/Ipopt_pkg
cd Ipopt_pkg
```

2. install ASL

```SHELL
git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
cd ThirdParty-ASL
sudo ./get.ASL
sudo ./configure
sudo make
sudo make install
cd ..
```

3. install HSL

download the [coinhsl](https://github.com/CHH3213/testCPP/blob/master/coinhsl.zip)

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

4. install MUMPS

```SHELL
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
sudo ./get.Mumps
sudo ./configure
sudo make
sudo make install
cd ..
```

5. install Ipopt

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

6. prefect the environment

```SHELL
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3
```
The Ipopt is installed successfully until now.

7. install the casadi use the bash

the bash is:
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

```SHELL
cd ~/
touch install-casadi.sh
sudo chmod +x install-casadi.sh
./install-casadi.sh
```
The Casadi is installed successfully until now.


