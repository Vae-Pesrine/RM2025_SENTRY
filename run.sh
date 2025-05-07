#! /bin/bash
killall gzserver
killall gzclient
# run the super terminal
terminator &
sleep 0.5

# set four windows
xdotool key Ctrl+Shift+o
xdotool key Ctrl+Tab
xdotool key Ctrl+Shift+e
xdotool key Ctrl+Tab
xdotool key Ctrl+Shift+e

xdotool key Ctrl+Tab
xdotool type "roslaunch sentry_bringup 01livox.launch"
xdotool key Return
sleep 4.0

xdotool key Ctrl+Tab
xdotool type "roslaunch sentry_bringup 02localize.launch"
xdotool key Return
sleep 3.0

xdotool key Ctrl+Tab
xdotool type "roslaunch sentry_bringup 03nav.launch"
xdotool key Return
sleep 1.0

# run the rviz in the end, otherwise the command can't be run properly because of the ui
xdotool key Ctrl+Tab
xdotool type "roslaunch sentry_bringup 04rviz.launch"
xdotool key Return