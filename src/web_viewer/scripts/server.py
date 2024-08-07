import http.server
import socketserver

import os 

PORT = 6564
SCRIPTS_DIR = os.path.dirname(os.path.realpath(__file__))
DIRECTORY = os.path.dirname(SCRIPTS_DIR) + '/html'

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, directory=DIRECTORY, **kwargs)

with socketserver.TCPServer(("", PORT), CustomHTTPRequestHandler) as httpd:
  print(f"Serving HTTP on port {PORT} from {DIRECTORY}")
  httpd.serve_forever()
