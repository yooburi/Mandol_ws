#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial, threading, time

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # 파라미터 선언
        self.declare_parameter('throttle_topic', '/throttle_cmd')
        self.declare_parameter('steer_cmd_topic', '/auto_steer_angle')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('startup_silence_sec', 3.0)   # ★ 시작 후 송신 차단 시간(초)

        throttle_topic = self.get_parameter('throttle_topic').get_parameter_value().string_value
        steer_topic    = self.get_parameter('steer_cmd_topic').get_parameter_value().string_value
        port           = self.get_parameter('port').get_parameter_value().string_value or DEFAULT_PORT
        baud           = self.get_parameter('baud').get_parameter_value().integer_value or DEFAULT_BAUD
        self.startup_silence_sec = float(self.get_parameter('startup_silence_sec').value)  # ★

        # 시리얼 핸들/락
        self.ser = None
        self._ser_lock = threading.Lock()
        self._stop = False

        # ★ 시작 시각 및 1회성 로그 플래그
        self._start_time = time.time()
        self._silence_logged = False

        # 재연결 스레드 (open)
        self._recon_th = threading.Thread(target=self._reconnect_loop, args=(port, baud), daemon=True)
        self._recon_th.start()

        # 수신 스레드 (아두이노 → 호스트) : 아두이노 Serial.print() 를 ROS 로그로 보여줌
        self._rx_th = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_th.start()

        # 구독자
        self.create_subscription(Float32, throttle_topic, self.cb_throttle, 10)
        self.create_subscription(Float32, steer_topic, self.cb_steer, 10)

        self.get_logger().info(
            f"Subscribed to {throttle_topic} and {steer_topic} → Serial({port}@{baud}), "
            f"startup_silence_sec={self.startup_silence_sec:.1f}s"   # ★
        )

    # 시리얼 열기/재연결 루프
    def _reconnect_loop(self, port, baud):
        while not self._stop:
            if self.ser is None:
                try:
                    self.get_logger().info(f"Opening serial: {port}@{baud}")
                    s = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=0.2)
                    time.sleep(0.2)  # 보드 리셋 안정화
                    with self._ser_lock:
                        self.ser = s
                    self.get_logger().info("Serial connected.")
                except Exception as e:
                    self.get_logger().warn(f"Serial open failed: {e}")
                    time.sleep(1.0)
            time.sleep(0.1)

    # 수신 루프: 아두이노에서 오는 디버그 문자열을 읽어 ROS 로그로 출력
    def _reader_loop(self):
        buf = b""
        while not self._stop:
            with self._ser_lock:
                s = self.ser
            if s is None:
                time.sleep(0.1)
                continue
            try:
                data = s.read(128)
                if not data:
                    time.sleep(0.01)
                    continue
                buf += data
                # \n 기준으로 라인 파싱
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    # \r 제거
                    line = line.replace(b'\r', b'')
                    text = line.decode('utf-8', errors='replace')
                    if text:
                        self.get_logger().info(f"RX: {text}")
            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}")
                with self._ser_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.2)

    # ★ 시작 후 송신 차단 여부
    def _in_startup_silence(self) -> bool:
        elapsed = time.time() - self._start_time
        if elapsed < self.startup_silence_sec:
            # 1회만 남은 시간 안내
            if not self._silence_logged:
                self._silence_logged = True
                remain = self.startup_silence_sec - elapsed
                self.get_logger().info(
                    f"Startup silence... (no serial writes for {remain:.1f}s more)"
                )
            return True
        return False

    # 한 줄 쓰기
    def _write_line(self, line: str):
        # ★ 시작 후 silence 동안은 송신 차단
        if self._in_startup_silence():
            return False
        with self._ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False

    # 토픽 콜백들 (TX 로깅 포함)
    def cb_throttle(self, msg: Float32):
        val = float(msg.data)
        if val > 1.0:  val = 1.0
        if val < -1.0: val = -1.0
        line = f"TH {val:.3f}\n"
        self.get_logger().info(f"TX: {line.strip()}")
        self._write_line(line)

    def cb_steer(self, msg: Float32):
        ang = float(msg.data)
        line = f"SA {ang:.3f}\n"
        self.get_logger().info(f"TX: {line.strip()}")
        self._write_line(line)

    def destroy_node(self):
        self._stop = True
        # 종료 대기
        try:
            self._recon_th.join(timeout=0.5)
            self._rx_th.join(timeout=0.5)
        except Exception:
            pass
        with self._ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()은 내부에서 두 번 호출되면 에러가 나므로 안전 가드
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
