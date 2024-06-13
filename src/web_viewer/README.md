# Requirements
```bash
sudo apt-get install ros-noetic-rosbridge-server
```

# Start Up Server
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
cd src/web_viewer/
```

```bash
python3 -m http.server
```

Which will launch on `localhost:9090` of host computer.

To visit the page on a different device, you have to be on a shared local network.
Assuming host computer is on Ubuntu, you can check your local IP by going to WiFi Settings.

Click on the gear button and grab the IPv4 Address, that's the IP to send to other to see the page.

Ex: `192.168.1.4:9090`
