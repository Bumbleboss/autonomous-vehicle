const url = {
  address: '192.168.43.15',
  port: 9090
}

// ROS instance
const ros = new ROSLIB.Ros({
  url: `ws://${url.address}:${url.port}`
});

// left side elements
const speed_elm = document.getElementById('speed');
const drive_elm = document.getElementById('mode');
const connect_elm = document.getElementById('connect');

// camera elements
const camera_elm = document.getElementById('camera');
const camera_toggle_elm = document.getElementById('camera_toggle');

// goal elements
const goal_elm = document.getElementById('goal');
const goal_dialog_elm = document.getElementById('goal_dialog');

// indicator elements
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

const goal_publisher = new ROSLIB.Topic({
  ros: ros,
  name: '/move_base/goal',
  messageType: 'move_base_msgs/MoveBaseActionGoal'
});

// driving modes
const drivingModes = {
  0: 'Manual Mode',
  1: 'Calibration Mode',
  2: 'Autonomous Mode',
  3: 'Steady State Mode',
  4: 'Calibration Mode'
};

// update driving mode
driving_listener.subscribe((msg) => {
  drive_elm.children[0].children[0].innerText = drivingModes[msg.data];
  
  if (msg.data == 1) {
    calibrate_elm.classList.remove('inactive');
    calibrate_elm.classList.add('pending');
  }

  if (msg.data == 4) {
    calibrate_elm.classList.remove('pending');
    calibrate_elm.classList.add('active');
  }
});

// update vehicle speed
odom_listener.subscribe((msg) => {
  let vel = msg.twist.twist.linear.z;
  speed_elm.children[0].innerText = `${vel} m/s`
})

// update detected objects text
obj_listener.subscribe((msg) => {
  let objs = msg.objects.length;
  objs_elm.children[0].innerText = objs
})

// toggle camera views
camera_elm.addEventListener('click', () => {
  camera_elm.classList.toggle('active');
  camera_toggle_elm.classList.toggle('show');
})

goal_elm.addEventListener('click', () => {
  goal_dialog_elm.showModal();
})

goal_dialog_elm.addEventListener('click', (e) => {
  if (e.target.nodeName === 'DIALOG')
    goal_dialog_elm.close()
})

goal_dialog_elm.children[0].addEventListener('submit', (e) => {
  e.preventDefault()

  const btn = e.submitter;
  if (btn.tagName == 'BUTTON') {
    goal_dialog_elm.close()
  } else {
    x_value = parseFloat(document.getElementById('goal_x').value)
    y_value = parseFloat(document.getElementById('goal_y').value)
    o_value = parseFloat(document.getElementById('goal_o').value)

    publish_goal(x_value, y_value, o_value)
  }
})

function publish_goal(x, y, z) {
  let current_time = new Date().getTime() / 1000.0;

  let secs = Math.floor(current_time);
  let nsecs = Math.floor((current_time - secs) * 1e9);

  // move_base_goal_message
  let move_base_goal = new ROSLIB.Message({
    header: {
      stamp: { secs, nsecs },
    },
    goal: {
      target_pose: {
        header: {
          stamp: { secs, nsecs },
          frame_id: 'map'
        },
        pose: {
          position: {
            x: x,
            y: y,
          },
          orientation: {
            z: z,
            w: Math.sqrt(1 - z * z)
          }
        }
      }
    }
  });

  goal_publisher.publish(move_base_goal);
}