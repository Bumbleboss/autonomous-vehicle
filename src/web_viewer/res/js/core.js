// ROS instance
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Adjust the WebSocket URL as needed
});

const drive_elm = document.getElementById('mode');
const connect_elm = document.getElementById('connect');
const calibrate_elm = document.getElementById('calibrate');

// event listeners for connection status
ros.on('connection', () => {
  connect_elm.classList.remove('inactive');
  connect_elm.classList.remove('pending');
  connect_elm.classList.add('active');
});

ros.on('error', (error) => {
  connect_elm.classList.add('inactive');
  connect_elm.classList.remove('pending');
  connect_elm.classList.remove('active');
});

ros.on('close', () => {
  connect_elm.classList.remove('inactive');
  connect_elm.classList.add('pending');
  connect_elm.classList.remove('active');
});

// driving mode and calibration subscriber
const driving_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/driving_mode',
  messageType: 'std_msgs/UInt8'
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
