#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time
import serial
from serial import SerialException


def recv_loop(port: str, baud: int, reconnect_interval: float):
    ser = None
    rx_buffer = bytearray()

    while True:
        # 自动重连
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(
                    port = port,
                    baudrate = baud,
                    timeout = 0.05,       # 小阻塞，避免空转占用CPU
                    write_timeout = 0.05,
                )
                rx_buffer.clear()
                print(f"[INFO] serial opened: {port} @ {baud}")
            except Exception as e:
                print(f"[WARN] open failed: {e}")
                time.sleep(reconnect_interval)
                continue

        try:
            waiting = ser.in_waiting
            if waiting <= 0:
                time.sleep(0.002)
                continue

            chunk = ser.read(waiting)
            if not chunk:
                continue

            rx_buffer.extend(chunk)

            # 按行解析（JSONL）
            while True:
                idx = rx_buffer.find(b"\n")
                if idx < 0:
                    break

                raw_line = bytes(rx_buffer[:idx])
                del rx_buffer[:idx + 1]

                line = raw_line.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    # 非JSON行可忽略，也可打印调试
                    continue

                # 协议过滤
                if msg.get("protocol") != "xrobot-nav-jsonl-v1":
                    continue

                msg_type = msg.get("msg_type")
                subtype = msg.get("subtype")
                seq = msg.get("seq")
                ts = msg.get("timestamp")
                data = msg.get("data", {})

                if msg_type == "imu" and subtype == "raw":
                    roll = float(data.get("roll", 0.0))
                    pitch = float(data.get("pitch", 0.0))
                    yaw = float(data.get("yaw", 0.0))
                    gx = float(data.get("gyro_x", 0.0))
                    gy = float(data.get("gyro_y", 0.0))
                    gz = float(data.get("gyro_z", 0.0))
                    ax = float(data.get("accel_x", 0.0))
                    ay = float(data.get("accel_y", 0.0))
                    az = float(data.get("accel_z", 0.0))

                    # print(
                    #     f"[IMU] seq={seq} ts={ts} "
                    #     f"rpy=({roll:.3f},{pitch:.3f},{yaw:.3f}) "
                    #     f"gyro=({gx:.3f},{gy:.3f},{gz:.3f}) "
                    #     f"acc=({ax:.3f},{ay:.3f},{az:.3f})"
                    # )
                elif msg_type == "event":
                    print(f"[EVENT] seq={seq} ts={ts} subtype={subtype} data={data}")
                else:
                    # 其他消息类型
                    print(f"[RX] type={msg_type} subtype={subtype} data={data}")

        except SerialException as e:
            print(f"[WARN] serial exception: {e}")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(reconnect_interval)
        except KeyboardInterrupt:
            print("\n[INFO] stopped by user")
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass
            break
        except Exception as e:
            print(f"[WARN] recv exception: {e}")


def main():
    parser = argparse.ArgumentParser(description="XRobot serial receive-only parser")
    parser.add_argument("--port", default="/dev/ttyS7")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--reconnect-interval", type=float, default=1.0)
    args = parser.parse_args()

    recv_loop(args.port, args.baud, args.reconnect_interval)


if __name__ == "__main__":
    main()