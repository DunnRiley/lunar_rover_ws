#!/usr/bin/env python3
"""
USAGE:
  from delay_queue import DelayQueue

  def my_send(device, speed, direction):
      ser.write(bytes([0xAA, device, speed, direction, 0x55]))

  dq = DelayQueue(send_cb=my_send, delay_sec=5.0)
  dq.enabled = True          # toggle at runtime
  dq.enqueue(0x05, 200, 0)   # queues a LEFT-drive command
  dq.stop()                  # call on shutdown
"""

import threading
import time
from collections import deque
from typing import Callable, Optional


class DelayQueue:
    """
    Thread-safe command delay queue.

    Parameters
    ----------
    send_cb : callable(device, speed, direction)
        Called when a command is released from the queue.
    delay_sec : float
        Hold time in seconds (default 5.0).
    on_enqueue : callable() | None
        Called each time a new command enters the queue (use to
        start the lap timer in the GUI).
    """

    def __init__(
        self,
        send_cb: Callable[[int, int, int], None],
        delay_sec: float = 5.0,
        on_enqueue: Optional[Callable[[], None]] = None,
    ):
        self._send_cb   = send_cb
        self._delay     = delay_sec
        self._on_enqueue = on_enqueue
        self._enabled   = False        # off by default; toggle via .enabled
        self._lock      = threading.Lock()
        self._queue: deque = deque()   # entries: (enqueue_time, device, speed, dir)
        self._running   = True
        self._thread    = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        self._enabled = bool(value)
        if not value:
            # Flush remaining delayed commands immediately on disable
            with self._lock:
                while self._queue:
                    _, device, speed, direction = self._queue.popleft()
                    try:
                        self._send_cb(device, speed, direction)
                    except Exception:
                        pass

    @property
    def delay_sec(self) -> float:
        return self._delay

    @delay_sec.setter
    def delay_sec(self, value: float):
        self._delay = max(0.0, float(value))

    def enqueue(self, device: int, speed: int, direction: int) -> None:
        """
        Submit a command.  If delay is disabled the command is sent
        synchronously; otherwise it is queued with the current timestamp
        and the on_enqueue callback is fired.
        """
        if not self._enabled:
            try:
                self._send_cb(device, speed, direction)
            except Exception:
                pass
            return

        with self._lock:
            self._queue.append((time.monotonic(), device, speed, direction))

        if self._on_enqueue is not None:
            try:
                self._on_enqueue()
            except Exception:
                pass

    def pending_count(self) -> int:
        with self._lock:
            return len(self._queue)

    def stop(self) -> None:
        """Shutdown background worker (call on program exit)."""
        self._running = False

    # ── Worker ────────────────────────────────────────────────────────────

    def _worker(self) -> None:
        while self._running:
            time.sleep(0.05)   # 20 Hz poll — low overhead
            if not self._enabled:
                continue
            now = time.monotonic()
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    enqueue_t, device, speed, direction = self._queue[0]
                    if now - enqueue_t < self._delay:
                        break      # front of queue not ready yet
                    self._queue.popleft()

                try:
                    self._send_cb(device, speed, direction)
                except Exception as e:
                    print(f'[DelayQueue] send error: {e}', flush=True)