#!/usr/bin/env python3
"""
receiver-rpi-servo.py
Raspberry Pi Servo Client for ESP32-S3 Transmitter

Connects to the transmitter on TCP port 5000, sends text registration
"ID:<robot_id>\n", then receives binary ControlPacket and HeartbeatPacket
frames. Same protocol as the ESP32-S2 (BS2) and FeatherS3 (Propeller)
receivers.

Binary packet format:
  ControlPacket (9 bytes):
    [0] 0xAA  start
    [1] 0x01  type = CONTROL
    [2] id_lo
    [3] id_hi
    [4] left    0..180 (90 = stop)
    [5] right   0..180 (90 = stop)
    [6] gripper 0=none, 1=open, 2=close
    [7] seq
    [8] checksum (XOR of bytes 0..7)

  HeartbeatPacket (6 bytes):
    [0] 0xAA  start
    [1] 0x02  type = HEARTBEAT
    [2] id_lo
    [3] id_hi
    [4] seq
    [5] checksum (XOR of bytes 0..4)

Features:
  - Auto-reconnect on any error or missed heartbeats
  - Non-blocking socket with select()
  - TCP keepalive enabled
  - Watchdog: auto-stop wheels if no valid command within N seconds
  - ID filtering: applies only if packet ID matches this robot's ID
"""

import os, sys, time, struct, socket, select, signal, argparse
from adafruit_servokit import ServoKit

# ========= Defaults =========
DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 5000
DEFAULT_ID   = int(os.environ.get("ROBOT_ID", "1"))

# ========= Heartbeat / timing =========
HB_TIMEOUT         = 8.0    # reconnect if no heartbeat for this long (seconds)
WATCHDOG_SECONDS    = 2.5    # stop wheels if no valid command within this time
RECONNECT_DELAY     = 2.0

# ========= Packet types =========
PKT_CONTROL   = 1
PKT_HEARTBEAT = 2

GRIP_NONE  = 0
GRIP_OPEN  = 1
GRIP_CLOSE = 2

# ========= Servo HAT settings =========
KIT = ServoKit(channels=16, address=0x40)

CH_GRIPPER = 0
CH_RIGHT   = 1
CH_LEFT    = 2

KIT.servo[CH_GRIPPER].set_pulse_width_range(500, 2500)
KIT.continuous_servo[CH_LEFT].set_pulse_width_range(1000, 2000)
KIT.continuous_servo[CH_RIGHT].set_pulse_width_range(1000, 2000)

GRIPPER_OPEN_ANGLE  = 0
GRIPPER_CLOSE_ANGLE = 180

# ========= Helpers =========

def deg_to_throttle(v):
    """Map 0..180 to -1..+1 with 90 = stop."""
    thr = (float(v) - 90.0) / 90.0
    return max(-1.0, min(1.0, thr))

def set_drive(left=None, right=None):
    if left is not None:
        KIT.continuous_servo[CH_LEFT].throttle = float(left)
    if right is not None:
        KIT.continuous_servo[CH_RIGHT].throttle = float(right)

def set_gripper(cmd):
    if cmd == GRIP_OPEN:
        KIT.servo[CH_GRIPPER].angle = GRIPPER_OPEN_ANGLE
        return "open", GRIPPER_OPEN_ANGLE
    elif cmd == GRIP_CLOSE:
        KIT.servo[CH_GRIPPER].angle = GRIPPER_CLOSE_ANGLE
        return "close", GRIPPER_CLOSE_ANGLE
    return "none", None

def stop_motors():
    KIT.continuous_servo[CH_LEFT].throttle = 0.0
    KIT.continuous_servo[CH_RIGHT].throttle = 0.0

def cleanup_and_exit(signum=None, frame=None):
    try:
        stop_motors()
    finally:
        sys.exit(0)

signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# ========= Checksum =========

def xor_checksum(data):
    x = 0
    for b in data:
        x ^= b
    return x

# ========= Networking =========

def set_tcp_keepalive(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, 0x10, 60)   # TCP_KEEPIDLE
        sock.setsockopt(socket.IPPROTO_TCP, 0x12, 10)   # TCP_KEEPINTVL
        sock.setsockopt(socket.IPPROTO_TCP, 0x11, 3)    # TCP_KEEPCNT
    except Exception:
        pass

def connect_and_register(host, port, robot_id, timeout=5.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    set_tcp_keepalive(s)
    s.settimeout(timeout)
    s.connect((host, port))
    print(f"[INFO] Connected to {host}:{port}")

    # Send registration (same as ESP32-S2 and FeatherS3)
    reg = f"ID:{robot_id}\n".encode()
    s.sendall(reg)
    print(f"[INFO] Sent registration: ID:{robot_id}")

    s.settimeout(0.0)  # non-blocking for select/recv
    return s

# ========= Binary packet parser =========

class PacketParser:
    """
    Streaming binary packet parser.
    Feed raw bytes via feed(), get parsed packets from parse().
    Handles partial packets across recv() calls.
    """
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)

    def parse(self):
        """Yield (pkt_type, pkt_id, payload_dict) tuples."""
        while len(self.buf) > 0:
            # Find 0xAA start marker
            if self.buf[0] != 0xAA:
                try:
                    idx = self.buf.index(0xAA)
                    self.buf = self.buf[idx:]
                except ValueError:
                    self.buf.clear()
                    return
                continue

            if len(self.buf) < 2:
                return  # need more data

            pkt_type = self.buf[1]

            if pkt_type == PKT_CONTROL:
                pkt_size = 9
            elif pkt_type == PKT_HEARTBEAT:
                pkt_size = 6
            else:
                # unknown type, skip this 0xAA
                self.buf = self.buf[1:]
                continue

            if len(self.buf) < pkt_size:
                return  # need more data

            pkt_data = bytes(self.buf[:pkt_size])

            # Validate checksum
            cs = xor_checksum(pkt_data[:pkt_size - 1])
            if cs != pkt_data[pkt_size - 1]:
                # bad checksum, skip this 0xAA
                self.buf = self.buf[1:]
                continue

            # Extract ID
            pkt_id = pkt_data[2] | (pkt_data[3] << 8)

            if pkt_type == PKT_CONTROL:
                payload = {
                    "left":    pkt_data[4],
                    "right":   pkt_data[5],
                    "gripper": pkt_data[6],
                    "seq":     pkt_data[7],
                }
                yield (PKT_CONTROL, pkt_id, payload)
            elif pkt_type == PKT_HEARTBEAT:
                yield (PKT_HEARTBEAT, pkt_id, {"seq": pkt_data[4]})

            # Consume packet
            self.buf = self.buf[pkt_size:]

# ========= Main loop =========

def main():
    parser_arg = argparse.ArgumentParser(
        description="RPi Servo client: binary protocol, port 5000"
    )
    parser_arg.add_argument("host", nargs="?", default=DEFAULT_HOST)
    parser_arg.add_argument("port", nargs="?", type=int, default=DEFAULT_PORT)
    parser_arg.add_argument("--id", type=int, default=DEFAULT_ID,
                            help="Robot ID (default from $ROBOT_ID or 1)")
    args = parser_arg.parse_args()

    my_id = int(args.id)
    host, port = args.host, int(args.port)

    print(f"[INFO] ROBOT_ID={my_id}")
    print(f"[INFO] Server={host}:{port}")
    print(f"[INFO] HB timeout={HB_TIMEOUT}s, Watchdog={WATCHDOG_SECONDS}s")
    print(f"[INFO] Gripper open={GRIPPER_OPEN_ANGLE} close={GRIPPER_CLOSE_ANGLE}")

    pkt_parser = PacketParser()
    last_cmd_ts = time.time()
    last_hb_ts  = time.time()
    s = None

    while True:
        try:
            if s is None:
                s = connect_and_register(host, port, my_id, timeout=5.0)
                pkt_parser = PacketParser()
                now = time.time()
                last_cmd_ts = now
                last_hb_ts  = now
                stop_motors()

            r, _, _ = select.select([s], [], [], 0.1)
            now = time.time()

            # Heartbeat timeout
            if now - last_hb_ts > HB_TIMEOUT:
                raise TimeoutError(
                    f"No heartbeat for {HB_TIMEOUT:.0f}s, reconnecting"
                )

            # Watchdog: stop wheels if stale
            if now - last_cmd_ts > WATCHDOG_SECONDS:
                stop_motors()
                last_cmd_ts = now

            if not r:
                continue

            try:
                chunk = s.recv(4096)
            except BlockingIOError:
                chunk = b""
            if not chunk:
                raise ConnectionError("Server closed connection")

            pkt_parser.feed(chunk)

            for pkt_type, pkt_id, payload in pkt_parser.parse():

                # Heartbeat: accept for any ID (same as all other receivers)
                if pkt_type == PKT_HEARTBEAT:
                    if pkt_id == my_id:
                        last_hb_ts = time.time()
                        print(f"[HB] seq={payload['seq']}")
                    continue

                # Control: filter by ID
                if pkt_type == PKT_CONTROL:
                    if pkt_id != my_id:
                        continue

                    left_v  = payload["left"]
                    right_v = payload["right"]
                    grip_v  = payload["gripper"]
                    seq     = payload["seq"]

                    # Drive wheels
                    left_thr  = deg_to_throttle(left_v)
                    right_thr = deg_to_throttle(right_v)
                    set_drive(left_thr, right_thr)

                    # Gripper
                    if grip_v == GRIP_OPEN or grip_v == GRIP_CLOSE:
                        mode, val = set_gripper(grip_v)
                        print(f"[CMD] L={left_v} R={right_v} G={mode}({val}) seq={seq}")
                    else:
                        print(f"[CMD] L={left_v} R={right_v} G=none seq={seq}")

                    last_cmd_ts = time.time()
                    last_hb_ts  = time.time()  # control packets also count as alive

        except Exception as e:
            print(f"[WARN] {e}. Reconnecting in {RECONNECT_DELAY}s...")
            try:
                if s:
                    s.close()
            except Exception:
                pass
            s = None
            stop_motors()
            time.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    main()
