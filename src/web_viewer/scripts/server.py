import http.server
import socketserver
import os

import rospy

PORT = 6564
SCRIPTS_DIR = os.path.dirname(os.path.realpath(__file__))
DIRECTORY = os.path.dirname(SCRIPTS_DIR) + '/html'

# Initialize ROS node (if necessary)
rospy.init_node('web_server', anonymous=True)

# Retrieve ROS parameters
rosbridge_address = rospy.get_param('~rosbridge_address', '192.168.43.15')
rosbridge_port = rospy.get_param('~rosbridge_port', 9090)

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def do_GET(self):
        if self.path == '/ros_config.js':
            self.send_response(200)
            self.send_header("Content-type", "application/javascript")
            self.end_headers()
            js_content = f"""
            const url = {{
                address: '{rosbridge_address}',
                port: {rosbridge_port}
            }}

            // ROS instance
            const ros = new ROSLIB.Ros({{
                url: `ws://${{url.address}}:${{url.port}}`
            }});

            ros.on('connection', function() {{
                console.log('Connected to websocket server.');
            }});

            ros.on('error', function(error) {{
                console.log('Error connecting to websocket server: ', error);
            }});

            ros.on('close', function() {{
                console.log('Connection to websocket server closed.');
            }});
            """
            self.wfile.write(js_content.encode('utf-8'))
        else:
            # Serve other files normally
            super().do_GET()

with socketserver.TCPServer(("", PORT), CustomHTTPRequestHandler) as httpd:
    print(f"Serving HTTP on port {PORT} from {DIRECTORY}")
    httpd.serve_forever()
