"""Pulls MJPEG streams from the Pi camera server in background threads."""

import threading
import time
import urllib.request

import cv2
import numpy as np


class MJPEGClient:
    def __init__(self, url: str, timeout: float = 5.0):
        self.url = url
        self.timeout = timeout
        self._frame = None
        self._ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"mjpeg")

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def get_frame(self):
        with self._lock:
            return self._frame, self._ts

    def _run(self):
        while not self._stop.is_set():
            try:
                stream = urllib.request.urlopen(self.url, timeout=self.timeout)
                buf = b""
                while not self._stop.is_set():
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b'\xff\xd8')
                    b = buf.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = buf[a:b + 2]
                        buf = buf[b + 2:]
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            with self._lock:
                                self._frame = img
                                self._ts = time.time()
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[MJPEGClient] {self.url}: {e}", flush=True)
                    time.sleep(1.0)
