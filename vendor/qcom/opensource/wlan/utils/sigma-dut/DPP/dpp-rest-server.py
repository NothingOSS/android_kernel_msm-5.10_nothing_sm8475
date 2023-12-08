#!/usr/bin/env python3
#
# Sigma Control API DUT (DPP REST API server)
# Copyright (c) 2022, Qualcomm Innovation Center, Inc.
# All Rights Reserved.
# Licensed under the Clear BSD license. See README for more details.

from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import os
import wpaspy

wpas_ctrl = '/var/run/wpa_supplicant'
ifname = None

def wpas_connect():
    ifaces = []
    if os.path.isdir(wpas_ctrl):
        try:
            ifaces = [os.path.join(wpas_ctrl, i) for i in os.listdir(wpas_ctrl)]
        except OSError as error:
            print("Could not find wpa_supplicant: %s", str(error))
            return None

    if len(ifaces) < 1:
        print("No wpa_supplicant control interface found")
        return None

    for ctrl in ifaces:
        if ifname and ifname not in ctrl:
            continue
        if os.path.basename(ctrl).startswith("p2p-dev-"):
            # skip P2P management interface
            continue
        try:
            print("Trying to use control interface " + ctrl)
            wpas = wpaspy.Ctrl(ctrl)
            return wpas
        except Exception as e:
            pass
    print("Could not connect to wpa_supplicant")
    return None

class DPP_REST_HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/dpp/bskey":
            self.send_response(404)
            self.end_headers()
            return

        if 'Content-Length' not in self.headers:
            self.send_response(411)
            self.end_headers()
            return

        content_len = int(self.headers['Content-Length'])
        body = self.rfile.read(content_len)

        try:
            data = json.loads(body)
        except:
            self.send_response(400)
            self.end_headers()
            self.wfile.write("Invalid JSON data".encode("ascii"))
            return

        if 'dppUri' not in data:
            print("Missing dppUri")
            self.send_response(400)
            self.end_headers()
            self.wfile.write("Missing dppUri".encode("ascii"))
            return

        wpas = wpas_connect()
        if wpas is None:
            self.send_response(503)
            self.end_headers()
            self.wfile.write("DPP Configurator not available".encode("ascii"))
            return

        res = wpas.request("DPP_QR_CODE " + data['dppUri'])

        if "UNKNOWN COMMAND" in res:
            self.send_response(503)
            self.end_headers()
            self.wfile.write("DPP Configurator not available".encode("ascii"))
            return

        if "FAIL" in res:
            self.send_response(400)
            self.end_headers()
            self.wfile.write("Invalid DPP URI".encode("ascii"))
            return

        id = int(res)
        with open("/tmp/dpp-rest-server.id", "w") as f:
            f.write(str(id))
        with open("/tmp/dpp-rest-server.uri", "w") as f:
            f.write(data['dppUri'])

        self.send_response(200)
        self.end_headers()

def main():
    # Bind to localhost only and use an external stunnel proxy to handle
    # TLS-PSK.
    httpd = HTTPServer(('localhost', 44444), DPP_REST_HTTPRequestHandler)
    httpd.serve_forever()

if __name__ == "__main__":
    main()
