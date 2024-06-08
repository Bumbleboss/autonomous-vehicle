// ROS instance
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Adjust the WebSocket URL as needed
});

// event listeners
ros.on('connection', () => {
  console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// Create a ROS topic subscriber
var drivingModeListener = new ROSLIB.Topic({
  ros: ros,
  name: '/driving_mode',
  messageType: 'std_msgs/UInt8'
});

// Map the UInt8 values to driving modes
var drivingModes = {
  0: 'Manual Mode',
  1: 'Calibration Mode',
  2: 'Autonomous Mode',
  3: 'Steady State Mode'
};

// // Subscribe to the topic
// drivingModeListener.subscribe(function(message) {
//   var mode = drivingModes[message.data] || 'Unknown Mode';
//   document.getElementById('driving-mode').innerText = mode;
//   console.log('Received message on /driving_mode: ' + message.data);
// });
