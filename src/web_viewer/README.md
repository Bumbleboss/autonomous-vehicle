# Requirements
```bash
sudo apt-get install ros-noetic-rosbridge-server
sudo apt-get install ros-noetic-web-video-server
```

# Usage
```bash
roslaunch web_viewer web.launch
```

Which will run a rosbridge instance, web server for streaming image urls and web server for page itself. 

Page can be accessed on `localhost:6564` of host computer.

To visit the page on a different device, you have to be on a local network.
Assuming host computer is on Ubuntu, you can check your local IP by going to WiFi Settings.

Click on the gear button and grab the IPv4 Address, that's the IP to send to other to see the page.

Ex: `192.168.1.4:6564`
