"""
dashboard.py - the control dashboard (the mock-up, live). A tiny web server inside follow.py; the page is dashboard.html.

    follow.py starts it, opens your browser, and the page shows the camera, the arena map, the lidar, the event log and the buttons
    (START / PAUSE / STOP, rover ID, target, colours, speed, manual drive).

Safety: it listens on 127.0.0.1 only (this computer), and every command must carry the random token printed in the URL, so
another web page open in your browser cannot press START for you.

Hooks (supplied by follow.py):
    snapshot()        -> a dict (JSON) with everything the page draws
    frame(name)       -> JPEG bytes of the latest picture ("external" or "onboard"), or None
    command(dict)     -> a dict reply; the commands are listed in follow.py
"""
import json
import os
import secrets
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")
SETTINGS_PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.html")
MAX_BODY = 4096


class Dashboard:
    def __init__(self, snapshot, frame, command, port=8765, host="127.0.0.1"):
        self.snapshot, self.frame, self.command = snapshot, frame, command
        self.token = secrets.token_hex(8)
        dash = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *a):          # keep the terminal for the rover's messages
                pass

            def _send(self, code, body, ctype="application/json", extra=None):
                if isinstance(body, str):
                    body = body.encode()
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def _ok_host(self):                  # also blocks DNS-rebinding: only our own address is a valid Host
                return (self.headers.get("Host") or "").split(":")[0] in ("127.0.0.1", "localhost", host)

            def do_GET(self):
                if not self._ok_host():
                    return self._send(403, '{"error":"bad host"}')
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    try:
                        html = open(PAGE, encoding="utf-8").read().replace("__TOKEN__", dash.token)
                    except OSError:
                        return self._send(500, "dashboard.html is missing next to dashboard.py", "text/plain")
                    return self._send(200, html, "text/html; charset=utf-8")
                if path in ("/settings", "/settings.html"):
                    try:
                        html = open(SETTINGS_PAGE, encoding="utf-8").read().replace("__TOKEN__", dash.token)
                    except OSError:
                        return self._send(500, "settings.html is missing next to dashboard.py", "text/plain")
                    return self._send(200, html, "text/html; charset=utf-8")
                if path == "/api/state":
                    try:
                        return self._send(200, json.dumps(dash.snapshot()))
                    except Exception as e:
                        return self._send(500, json.dumps({"error": repr(e)}))
                if path == "/frame.jpg":                       # one picture (the page polls this; no long-lived stream to hang)
                    q = self.path.split("?", 1)[1] if "?" in self.path else ""
                    name = "onboard" if "cam=onboard" in q else "external"
                    jpg = dash.frame(name)
                    if jpg is None:
                        return self._send(204, b"", "image/jpeg")
                    return self._send(200, jpg, "image/jpeg")
                return self._send(404, '{"error":"not found"}')

            def do_POST(self):
                if not self._ok_host() or self.headers.get("X-Token") != dash.token:
                    return self._send(403, '{"error":"bad token"}')
                n = int(self.headers.get("Content-Length") or 0)
                if n > MAX_BODY:
                    return self._send(413, '{"error":"too big"}')
                try:
                    cmd = json.loads(self.rfile.read(n) or b"{}")
                    if not isinstance(cmd, dict):
                        raise ValueError("not an object")
                except ValueError:
                    return self._send(400, '{"error":"bad json"}')
                try:
                    return self._send(200, json.dumps(dash.command(cmd) or {"ok": True}))
                except Exception as e:
                    return self._send(500, json.dumps({"error": repr(e)}))

        self.server = None
        for p in range(port, port + 20):               # the first free port from 8765 up
            try:
                self.server = ThreadingHTTPServer((host, p), Handler)
                self.port = p
                break
            except OSError:
                continue
        if self.server is None:
            raise OSError(f"no free port from {port} to {port + 19}")
        self.server.daemon_threads = True
        self.url = f"http://{host}:{self.port}/"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def open_browser(self):
        """Open the dashboard in the default browser. webbrowser.open() often says False or does nothing (no default browser set, WSL, remote
        sessions), so after it the operating system's own opener is tried. Returns True if something was started."""
        import subprocess
        import sys
        try:
            if webbrowser.open(self.url, new=2):
                return True
        except Exception:
            pass
        try:
            if sys.platform.startswith("win"):
                os.startfile(self.url)                                       # the Windows default-browser association
                return True
            cmd = ["open", self.url] if sys.platform == "darwin" else ["xdg-open", self.url]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass
