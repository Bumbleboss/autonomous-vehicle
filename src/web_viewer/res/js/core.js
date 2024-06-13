// ROS instance
const ros = new ROSLIB.Ros({
  url: 'ws://10.6.3.26:9090'  // Adjust the WebSocket URL as needed
});

const speed_elm = document.getElementById('speed');
const drive_elm = document.getElementById('mode');
const connect_elm = document.getElementById('connect');
const calibrate_elm = document.getElementById('calibrate');
const objs_elm = document.getElementById('objects');

// event listeners for connection status
ros.on('connection', () => {
  connect_elm.classList.remove('inactive');
  connect_elm.classList.add('active');
});

ros.on('error', (error) => {
  connect_elm.classList.add('inactive');
  connect_elm.classList.remove('active');
});

ros.on('close', () => {
  connect_elm.classList.add('inactive');
  connect_elm.classList.remove('active');
});

// driving mode and calibration subscriber
const driving_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/driving_mode',
  messageType: 'std_msgs/UInt8'
});

const odom_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/zed2/zed_node/odom',
  messageType: 'nav_msgs/Odometry'
});


const obj_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/zed2/zed_node/obj_det/objects',
  messageType: 'zed_interfaces/ObjectsStamped'
});

// driving modes
const drivingModes = {
  0: 'Manual Mode',
  1: 'Calibration Mode',
  2: 'Autonomous Mode',
  3: 'Steady State Mode'
};

// update driving mode
driving_listener.subscribe((msg) => {
  drive_elm.children[0].children[0].innerText = drivingModes[msg.data];
  
  if (msg.data == 1) {
    calibrate_elm.classList.remove('pending');
    calibrate_elm.classList.add('active');
  }
});

odom_listener.subscribe((msg) => {
  let vel = msg.twist.twist.linear.z;
  speed_elm.children[0].innerText = `${vel} m/s`
})

obj_listener.subscribe((msg) => {
  let objs = msg.objects.length;
  objs_elm.children[0].innerText = objs
})
