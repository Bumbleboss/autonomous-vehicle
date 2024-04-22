# Autonomous Vehicle

to run GPS plugins 
you should run this command
```
sudo apt-get install ros-noetic-hector-gazebo-plugins
```

and to make sure ackermann_steering plugin works
you should run this command 
```
sudo apt install ros-noetic-ros-control
sudo apt install ros-noetic-ros-controllers
sudo apt install ros-noetic-rqt-robot-steering
sudo apt install ros-noetic-ackermann-steering-controller
sudo apt-get install ros-noetic-gazebo-ros-control
```


---

## Running GPS in Real Time (NEO-M8N-0-10)

To utilize the NEO-M8N-0-10 GPS module in real-time, follow the steps below:

1. Install the necessary ROS package for the NMEA GPS driver:

```
sudo apt-get install ros-neotic-nmea-navsat-driver
```

2. Wait for approximately 5 minutes for the GPS module to initialize.

3. Launch the GPS driver using one of the following commands:
   - Specify the port directly:

```
roslaunch nmea_navsat_driver nmea_serial_driver.launch port:=/dev/ttyACM0 
```

-or Use a pre-configured launch file:

```
roslaunch historia GPS.launch
```

**Note:** Make sure to adjust the port name (/dev/ttyACM0) based on your system configuration.

4. If you encounter a permission denied error, grant access to the GPS port by running:

```
sudo chmod a+rw /dev/ttyACM0
```

5. Verify the published GPS messages by echoing the /fix topic:
```
rostopic echo /fix
```

6. To validate the accuracy of the GPS data, you can use online tools like [GPS Coordinates](https://gps-coordinates.org/).

---

