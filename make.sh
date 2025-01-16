#!/bin/bash

# 判断输入参数个数
if [ $# -eq 0 ]; then
    # 没有输入参数，执行 catkin_make
    catkin_make
elif [ $# -eq 1 ]; then
    # 输入要编译的功能包的名字 catkin_make -DCATKIN_WHITELIST_PACKAGES="name"
    catkin_make -DCATKIN_WHITELIST_PACKAGES="$1"
else
    echo "参数个数错误！"
    echo "用法："
    echo "    ./make.sh               # 执行 catkin_make"
    echo "    ./make.sh name          # 执行 catkin_make -DCATKIN_WHITELIST_PACKAGES=\"name\""
fi