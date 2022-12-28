#!/bin/env python3
import argparse
from datetime import datetime
import glob
from http import HTTPStatus
import http.server
import os
import shutil
import socket
import ssl
import sys
import threading


_Lock = threading.Lock()


def get_compile_datetime_from_bin(filename):
    """
    get compile time from ESP32 bin file
    Parameters
    ----------
    filename: str | bytes | PathLike[str] | PathLike[bytes] | int
    """
    # https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/firmware-image-format.html
    with open(filename, 'rb') as f:
        data = f.read()
    return data[0x80:0x8B].decode()+" "+data[0x70:0x78].decode()


class Server(http.server.ThreadingHTTPServer):
    """
    Parameters
    ----------
    images_dir: str | bytes | PathLike[str] | PathLike[bytes]
    """

    # allow for rapid stop/start cycles during debugging
    # Assumption is, that no other process will start listening on `port`
    # during restart of this script
    allow_reuse_address = True

    # allow IPv4 and IPv6
    address_family = socket.AF_INET6

    def __init__(self, images_dir, *args, logfile_name=None,  **kwargs):
        self.images_dir = images_dir
        self.logfile_name = logfile_name
        super().__init__(*args, **kwargs)


# noinspection PyPep8Naming
class RequestHandler(http.server.BaseHTTPRequestHandler):
    """
    Requesthandler providing *.bin files and virtual *.date files
    """
    server: Server

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def send_exception(self, e: Exception):
        if isinstance(e, OSError):
            self.send_error(HTTPStatus.NOT_FOUND, str(e))
        else:
            self.send_error(HTTPStatus.BAD_REQUEST, str(e))

    def do_GET(self):
        if self.path.endswith('date'):  # send build-date of firmware
            try:
                timestamp = get_compile_datetime_from_bin(os.path.join(
                    self.server.images_dir, self.path[1:-4] + "bin"))
                data = timestamp.encode()

                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Length", len(data))
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(data)

            except Exception as e:
                self.send_exception(e)

        elif self.path.endswith('bin'):  # send firmware image
            try:
                bin_filename = os.path.join(
                    self.server.images_dir, self.path[1:])

                self.send_response(HTTPStatus.OK)
                with open(bin_filename, 'rb') as f:
                    fs = os.fstat(f.fileno())
                    self.send_header("Content-Length", str(fs[6]))
                    self.send_header(
                        "Content-Type", "application/octet-stream")
                    self.end_headers()
                    shutil.copyfileobj(f, self.wfile)

                with _Lock:
                    with open(self.server.logfile_name, 'a') as f:
                        f.write(f'{datetime.now().isoformat()} {self.path}\n')

            except Exception as e:
                self.send_exception(e)

        # send JSON dictionary of available firmware images with build dates
        elif self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write("{\n".encode())
            for imagename in glob.glob(os.path.join(self.server.images_dir, "*.bin")):
                timestamp = get_compile_datetime_from_bin(imagename)
                imagename = os.path.basename(imagename)
                self.wfile.write(
                    f"\"{imagename}\":\"{timestamp}\",\n".encode())
            self.wfile.write("}".encode())

        else:
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")


def setup_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument('--images-dir', type=str)
    parser.add_argument('--port', type=int, default=8001)
    parser.add_argument('--certfile', "-c", type=str)
    parser.add_argument('--keyfile', "-k", type=str)
    parser.add_argument('--logfile', '-l', type=str)
    return parser


if __name__ == '__main__':
    p = setup_parser()
    argv = p.parse_args()

    with Server(os.path.realpath(argv.images_dir),
                ("", argv.port),
                RequestHandler,
                logfile_name=argv.logfile) as httpd:
        if argv.certfile:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            try:
                context.load_cert_chain(argv.certfile, argv.keyfile)
            except FileNotFoundError as e:
                print(
                    "Either certfile or keyfile not found. You can generate "
                    f"them using\nopenssl req -new -x509 -keyout {argv.keyfile}"
                    f" -out {argv.certfile} -days 3650 -nodes",
                    file=sys.stderr)
                raise e
            context.check_hostname = False
            httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
        httpd.serve_forever()
