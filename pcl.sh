#!/bin/bash

# 
echo -e "\e[32m--------Downloading pcl-1.12.0--------\e[0m"
wget -O pcl-1.12.0.zip https://github.com/PointCloudLibrary/pcl/archive/pcl-1.12.0.zip
unzip pcl-1.12.0.zip
sudo mv pcl-pcl-1.12.0 pcl-1.12.0
sudo rm -rf pcl-1.12.0.zip
echo -e "\e[32m--------Download done with source code in folder pcl-1.12.0--------\e[0m"


echo -e "\e[32m--------Compile source code--------\e[0m"
sudo mv pcl-1.12.0 /usr/local/include
cd /usr/local/include
sudo mkdir build
cd build
sudo cmake .. 
sudo make -j4   
sudo make install
cd /usr/local/include
sudo rm -rf pcl-1.12.0
echo -e "\e[32m--------Compile done--------\e[0m"


echo -e "\e[32m--------Change the default path of pcl--------\e[0m"
cd /opt/ros/noetic/share/pcl_ros/cmake
sudo chmod 777 pcl_rosConfig.cmake
sed -i 's|/usr/include/pcl-1.10|/usr/local/include/pcl-1.12|g' pcl_rosConfig.cmake
cd /opt/ros/noetic/share/pcl_conversions/cmake
sudo chmod 777 pcl_conversionsConfig.cmake  
sed -i 's|/usr/include/pcl-1.10|/usr/local/include/pcl-1.12|g' pcl_rosConfig.cmake

SOURCE_DIR = "/usr/local/lib"
TARGET_DIR = "/usr/lib/x86_64-linux-gnu"
for lib in $(find "$SOURCE_DIR" -name "libpcl*.so*"); do
    lib_name=$(basename "$lib")
    sudo ln -sf "$lib" "$TARGET_DIR/$lib_name"
    echo "Created symlink for $lib_name in $TARGET_DIR"
done
echo "All pcl libraries have been linked successfully."
echo -e "\e[32m--------Change done--------\e[0m"