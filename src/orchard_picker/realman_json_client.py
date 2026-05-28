import json
import socket
import threading
import time

from orchard_picker.protocol import dumps_compact


class JsonFrameClient:
    """Thread-safe line-framed JSON TCP client for RealMan controllers."""

    def __init__(
        self,
        host,
        port,
        connect_timeout=3.0,
        recv_timeout=0.2,
        rx_callback=None,
        tx_callback=None,
        error_callback=None,
        max_messages=500,
    ):
        self.host = host
        self.port = int(port)
        self.connect_timeout = float(connect_timeout)
        self.recv_timeout = float(recv_timeout)
        self.rx_callback = rx_callback
        self.tx_callback = tx_callback
        self.error_callback = error_callback
        self.max_messages = int(max_messages)

        self._socket = None
        self._reader = None
        self._stop = threading.Event()
        self._connected = False
        self._messages = []
        self._seq = 0
        self._condition = threading.Condition()
        self._send_lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self._command_lock = threading.Lock()

    @property
    def connected(self):
        return self._connected

    def connect(self):
        with self._connect_lock:
            if self._connected and self._socket is not None:
                return

            self.close()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.connect_timeout)
            sock.connect((self.host, self.port))
            sock.settimeout(self.recv_timeout)

            self._socket = sock
            self._stop.clear()
            self._connected = True
            self._reader = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader.start()

    def close(self):
        self._stop.set()
        sock = self._socket
        self._socket = None
        self._connected = False
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def send(self, payload):
        self.connect()
        text = dumps_compact(payload)
        frame = (text + "\r\n").encode("utf-8")
        with self._send_lock:
            if self._socket is None:
                raise RuntimeError("socket is not connected")
            self._socket.sendall(frame)
        if self.tx_callback:
            self.tx_callback(text)
        return text

    def send_command(
        self,
        payload,
        ack_predicate=None,
        ack_timeout=5.0,
        wait_predicate=None,
        wait_timeout=60.0,
    ):
        with self._command_lock:
            with self._condition:
                start_seq = self._seq

            self.send(payload)

            ack = None
            if ack_predicate is not None:
                ack = self.wait_for(ack_predicate, start_seq, ack_timeout)
                if ack is None:
                    return False, "timeout waiting for command response", None, None
                if not message_indicates_success(ack):
                    return False, "command response reports failure", ack, None

            done = None
            if wait_predicate is not None:
                done = self.wait_for(wait_predicate, start_seq, wait_timeout)
                if done is None:
                    return False, "timeout waiting for trajectory completion", ack, None
                if not bool(done.get("trajectory_state", False)):
                    return False, "trajectory completion reports failure", ack, done

            return True, "ok", ack, done

    def wait_for(self, predicate, start_seq, timeout):
        deadline = time.time() + float(timeout)
        cursor = 0
        while True:
            with self._condition:
                for seq, message, _line in self._messages[cursor:]:
                    if seq <= start_seq:
                        continue
                    if predicate(message):
                        return message
                cursor = len(self._messages)

                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._condition.wait(min(remaining, 0.2))

    def _append_message(self, message, raw_line):
        with self._condition:
            self._seq += 1
            self._messages.append((self._seq, message, raw_line))
            if len(self._messages) > self.max_messages:
                self._messages = self._messages[-self.max_messages :]
            self._condition.notify_all()

        if self.rx_callback:
            self.rx_callback(raw_line)

    def _reader_loop(self):
        buffer = b""
        while not self._stop.is_set():
            sock = self._socket
            if sock is None:
                break
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                self._report_error("socket receive error: {}".format(exc))
                break

            if not data:
                self._report_error("socket closed by peer")
                break

            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    text = line.decode("utf-8")
                    message = json.loads(text)
                except (UnicodeDecodeError, ValueError) as exc:
                    self._report_error("invalid JSON frame: {}".format(exc))
                    continue
                self._append_message(message, text)

        self._connected = False

    def _report_error(self, message):
        if self.error_callback:
            self.error_callback(message)


def message_indicates_success(message):
    for key in ("receive_state", "state", "set_state", "write_state", "read_state"):
        value = message.get(key)
        if isinstance(value, bool):
            return value
    return True


def command_ack(command_name):
    return lambda msg: msg.get("command") == command_name


def current_arm_state_ack(msg):
    return (
        msg.get("command") == "get_current_arm_state"
        or msg.get("state") == "current_arm_state"
        or "arm_state" in msg
    )


def gripper_ack(msg):
    return msg.get("command") in (
        "set_gripper",
        "set_gripper_release",
        "set_gripper_pick",
        "set_gripper_pick_on",
        "set_gripper_position",
    )


def trajectory_done(device):
    return (
        lambda msg: msg.get("state") == "current_trajectory_state"
        and int(msg.get("device", -1)) == int(device)
    )
