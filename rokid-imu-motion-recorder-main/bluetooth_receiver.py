#!/usr/bin/env python3    sudo python3 bluetooth_receiver.py AC:86:D1:54:A4:A9 5
MODEL_PATH = "model/vosk-model-small-cn-0.22"
"""
经典蓝牙 (RFCOMM) IMU+音频 数据接收器 - PC端
使用经典蓝牙接收来自Android设备的原始IMU数据（加速度计+陀螺仪）和麦克风音频
数据格式:
- IMU: IMU,ax,ay,az,gx,gy,gz
- Audio: AUD,<base64 PCM>
- Audio config: AUDCFG,sampleRate,channels,sampleWidthBytes
- Image: IMG,<base64 JPEG>
"""

import sys
import os
import csv
import base64
import socket
import time
import wave
import argparse
import json
import queue
import threading
from array import array
from datetime import datetime

try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except Exception:
    cv2 = None
    np = None
    CV_AVAILABLE = False

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    Image = None
    ImageDraw = None
    ImageFont = None
    PIL_AVAILABLE = False

# 自定义 UUID (必须与Android端一致)
CUSTOM_UUID = "96f66030-50c9-11ee-be56-0242ac120002"
FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/usr/share/fonts/truetype/arphic/ukai.ttc",
]

def map_command(text, last_sent):
    text_nospace = text.replace(" ", "")
    if text_nospace == "助力增加":
        value = "1"
    elif text_nospace == "助力减小":
        value = "2"
    else:
        return False, None, last_sent
    if value != last_sent:
        return True, value, value
    return False, None, last_sent

class BluetoothClassicReceiver:
    def __init__(self, save_to_file=True, enable_analysis=False, model_path=None, display=None):
        self.save_to_file = save_to_file
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None
        self.data_count = 0
        self.sock = None
        self.running = False
        self.first_data_received = False
        self.last_time = 0
        self.audio_file = None
        self.audio_filename = None
        self.audio_bytes_received = 0
        self.audio_sample_rate = 8000
        self.audio_channels = 1
        self.audio_sample_width = 2
        self.first_audio_received = False
        self.enable_analysis = enable_analysis
        self.model_path = model_path
        self.audio_analyzer = None
        self.display = display
        self.stop_requested = False
        self.last_sent = ""
        self.image_dir = None
        self.image_count = 0
        
        if self.save_to_file:
            if not os.path.exists('imu_data'):
                os.makedirs('imu_data')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_filename = f'imu_data/bt_classic_imu_{timestamp}.csv'
            self.audio_filename = f'imu_data/bt_audio_{timestamp}.wav'
            self.image_dir = f'imu_data/bt_images_{timestamp}'
            print(f"数据将保存到: {self.csv_filename} (收到数据后创建)")
            print(f"音频将保存到: {self.audio_filename} (收到音频后创建)")
            print(f"Images will be saved to: {self.image_dir} (created on first frame)")
    
    def find_service_port(self, address):
        """查找服务对应的 RFCOMM 端口"""
        print(f"正在查询服务端口 (UUID: {CUSTOM_UUID})...")
        try:
            # 尝试使用 pybluez 查找服务
            import bluetooth
            services = bluetooth.find_service(address=address, uuid=CUSTOM_UUID)
            
            if len(services) > 0:
                match = services[0]
                port = match["port"]
                name = match["name"]
                host = match["host"]
                print(f"找到服务 '{name}' 在主机 {host} 的端口 {port}")
                return port
            else:
                print("未找到匹配的服务。")
                print("尝试使用默认端口 1 (可能失败)...")
                return 1
        except ImportError:
            print("警告: 未安装 pybluez 模块，无法自动查找端口。")
            print("将尝试默认端口 1。如果连接失败，请安装 pybluez: pip install pybluez")
            return 1
        except Exception as e:
            print(f"服务查询失败: {e}")
            return 1

    def connect(self, address, port=None):
        while not self.stop_requested: # 外层循环：断线重连
            # 获取正确的端口
            current_port = port
            if current_port is None:
                current_port = self.find_service_port(address)
            
            print(f"正在连接到 {address} (端口 {current_port})...")
            
            connected = False
            self.sock = None
            
            # 内层循环：单次连接尝试
            while not self.stop_requested:
                try:
                    # 优先使用原生 Socket 避免 PyBluez 占用 rfcomm 设备
                    self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                    self.sock.settimeout(5)
                    self.sock.connect((address, current_port))
                    self.sock.settimeout(None)
                    connected = True
                    break
                except Exception:
                    # 回退尝试 PyBluez，部分环境需要
                    try:
                        import bluetooth
                        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                        self.sock.settimeout(5)
                        self.sock.connect((address, current_port))
                        self.sock.settimeout(None)
                        connected = True
                        break
                    except Exception as e2:
                        if self.sock:
                            try: self.sock.close()
                            except: pass
                        self.sock = None
                        print(f"连接失败: {e2}。 3秒后重试...")
                        time.sleep(3)
                        continue

                except Exception as e:
                    print(f"连接失败: {e}。 3秒后重试...")
                    if self.sock:
                        try: self.sock.close()
                        except: pass
                    self.sock = None
                    time.sleep(3)
                    # 继续下一次循环尝试
            
            if connected:
                print("连接成功!")
                try:
                    self.sock.send(b"HELLO_FROM_PC\n")
                except:
                    pass
                
                self.running = True
                self.receive_loop() # 进入接收循环，直到断开
                
                print("连接已断开，准备重连...")
                time.sleep(2)
                if self.stop_requested:
                    break
            
            # 如果 receive_loop 返回，说明连接断开了，外层循环会再次执行 connect

    def receive_loop(self):
        print("开始接收数据... (按 Ctrl+C 停止)")
        buffer = ""
        try:
            while self.running and not self.stop_requested:
                data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                if not data:
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if not line.strip(): continue
                    
                    parsed = self.parse_line(line)
                    if not parsed:
                        continue

                    kind, payload = parsed
                    if kind == "imu":
                        if not self.first_data_received:
                            self.first_data_received = True
                            if self.save_to_file:
                                self.create_csv_file()
                        
                        self.data_count += 1
                        self.display_data(payload)
                        if self.save_to_file:
                            self.save_data(payload)
                    elif kind == "audio_config":
                        self.update_audio_config(*payload)
                    elif kind == "audio":
                        self.handle_audio_chunk(payload)
                        if not self.first_audio_received:
                            self.first_audio_received = True
                            if self.save_to_file:
                                self.create_audio_file()
                        if self.save_to_file:
                            self.save_audio_data(payload)
                    elif kind == "image":
                        if self.save_to_file:
                            self.save_image_data(payload)
                            
        except KeyboardInterrupt:
            print("\n用户停止")
        except Exception as e:
            print(f"\n连接断开: {e}")
        finally:
            self.close()

    def close(self):
        self.running = False
        if self.sock:
            self.sock.close()
        if self.csv_file:
            self.csv_file.close()
            print(f"\n已保存 {self.data_count} 条数据")
        if self.audio_file:
            self.audio_file.close()
            print(f"\n已保存音频 {self.audio_bytes_received} 字节")
        if self.audio_analyzer:
            self.audio_analyzer.stop()
        if self.image_count > 0:
            print(f"\nSaved {self.image_count} images")

    def stop(self):
        self.stop_requested = True
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def parse_line(self, line):
        line = line.strip()
        if not line:
            return None

        if line.startswith("AUDCFG,"):
            parts = line.split(',')
            if len(parts) >= 4:
                try:
                    sample_rate = int(parts[1])
                    channels = int(parts[2])
                    sample_width = int(parts[3])
                except ValueError:
                    return None
                return ("audio_config", (sample_rate, channels, sample_width))
            return None

        if line.startswith("AUD,"):
            payload = line[4:]
            if not payload:
                return None
            try:
                return ("audio", base64.b64decode(payload))
            except Exception:
                return None

        if line.startswith("IMG,"):
            payload = line[4:]
            if not payload:
                return None
            try:
                return ("image", base64.b64decode(payload))
            except Exception:
                return None

        if line.startswith("IMU,"):
            line = line[4:]

        imu_data = self.parse_imu_data(line)
        if imu_data:
            return ("imu", imu_data)
        return None

    def parse_imu_data(self, line):
        try:
            parts = line.strip().split(',')
            if len(parts) == 6:
                return {
                    'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                    'accelX': float(parts[0]),
                    'accelY': float(parts[1]),
                    'accelZ': float(parts[2]),
                    'gyroX': float(parts[3]),
                    'gyroY': float(parts[4]),
                    'gyroZ': float(parts[5])
                }
            return None
        except ValueError:
            return None

    def update_audio_config(self, sample_rate, channels, sample_width):
        if self.audio_file:
            print("音频文件已创建，忽略新的音频配置")
            return
        if self.audio_analyzer and self.audio_analyzer.sample_rate != sample_rate:
            print("音频采样率变化，重启识别器")
            self.audio_analyzer.stop()
            self.audio_analyzer = None
        self.audio_sample_rate = sample_rate
        self.audio_channels = channels
        self.audio_sample_width = sample_width
        print(f"收到音频配置: {sample_rate}Hz, {channels}ch, {sample_width * 8}bit")
        self.ensure_audio_analyzer()

    def create_csv_file(self):
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            headers = ['Timestamp', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ']
            self.csv_writer.writerow(headers)
            print(f"CSV文件已创建: {self.csv_filename}")
        except Exception as e:
            print(f"创建CSV文件失败: {e}")

    def create_audio_file(self):
        try:
            self.audio_file = wave.open(self.audio_filename, 'wb')
            self.audio_file.setnchannels(self.audio_channels)
            self.audio_file.setsampwidth(self.audio_sample_width)
            self.audio_file.setframerate(self.audio_sample_rate)
            print(f"音频文件已创建: {self.audio_filename}")
        except Exception as e:
            print(f"创建音频文件失败: {e}")

    def save_data(self, data):
        if self.csv_writer:
            try:
                row = [
                    data['timestamp'],
                    data['accelX'], data['accelY'], data['accelZ'],
                    data['gyroX'], data['gyroY'], data['gyroZ']
                ]
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            except Exception as e:
                print(f"写入数据失败: {e}")

    def save_audio_data(self, data):
        if self.audio_file:
            try:
                self.audio_file.writeframes(data)
                self.audio_bytes_received += len(data)
            except Exception as e:
                print(f"写入音频失败: {e}")

    def create_image_dir(self):
        if not self.image_dir:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.image_dir = os.path.join('imu_data', f'bt_images_{timestamp}')
        try:
            os.makedirs(self.image_dir, exist_ok=True)
        except Exception as e:
            print(f"Failed to create image directory: {e}")
            self.image_dir = None

    def save_image_data(self, data):
        if not data:
            return
        if not self.image_dir:
            self.create_image_dir()
        if not self.image_dir:
            return
        try:
            self.image_count += 1
            filename = os.path.join(self.image_dir, f"frame_{self.image_count:06d}.jpg")
            with open(filename, 'wb') as f:
                f.write(data)
        except Exception as e:
            print(f"Failed to write image: {e}")

    def handle_audio_chunk(self, data):
        if self.display:
            level = self.calculate_audio_level(data)
            self.display.update_audio_level(level)
        if self.enable_analysis:
            self.ensure_audio_analyzer()
            if self.audio_analyzer:
                self.audio_analyzer.enqueue(data)

    def ensure_audio_analyzer(self):
        if self.audio_analyzer or not self.enable_analysis:
            return
        if not self.model_path:
            print("未配置模型路径，跳过音频分析")
            return
        self.audio_analyzer = AudioAnalyzer(
            model_path=self.model_path,
            sample_rate=self.audio_sample_rate,
            on_result=self.on_audio_result,
            on_partial=self.on_audio_partial,
        )
        if not self.audio_analyzer.start():
            self.audio_analyzer = None

    def on_audio_result(self, text):
        if not text:
            return
        matched = self.process_recognized_text(text, is_final=True)
        if not matched:
            print(f"\n识别结果: {text}")

    def on_audio_partial(self, text):
        if not text:
            return
        matched = self.process_recognized_text(text, is_final=False)
        if not matched:
            print(f"\n识别中: {text}")

    def process_recognized_text(self, text, is_final):
        if self.display:
            return self.display.handle_text(text, is_final=is_final)
        matched, value, new_last_sent = map_command(text, self.last_sent)
        if matched:
            self.last_sent = new_last_sent
            print(f"\n命令: {value}")
            return True
        return False

    def calculate_audio_level(self, data):
        if not data:
            return 0
        if self.audio_sample_width == 2:
            samples = array('h')
            samples.frombytes(data)
            if not samples:
                return 0
            return int(max(abs(value) for value in samples))
        if self.audio_sample_width == 1:
            return int(max(abs(value - 128) for value in data))
        return 0

    def display_data(self, data):
        current_time = time.time()
        if current_time - self.last_time >= 0.1:  # 限制刷新率，避免刷屏太快
            self.last_time = current_time
            print(f"\r[{data['timestamp']}] "
                  f"Acc: {data['accelX']:.2f}, {data['accelY']:.2f}, {data['accelZ']:.2f} | "
                  f"Gyro: {data['gyroX']:.2f}, {data['gyroY']:.2f}, {data['gyroZ']:.2f}", end="")
            sys.stdout.flush()

class DisplayManager:
    def __init__(self, window_title="Command", hold_seconds=2.0):
        self.window_title = window_title
        self.hold_seconds = hold_seconds
        self.state_lock = threading.Lock()
        self.latest_partial = ""
        self.latest_level = 0
        self.last_command = ""
        self.last_command_ts = 0.0
        self.last_sent = ""
        self.font_path = None
        self.font_cache = {}
        self.prepared = False

    def prepare(self):
        if self.prepared:
            return True
        if not CV_AVAILABLE or cv2 is None or np is None:
            print("未安装 opencv-python 或 numpy，无法显示窗口")
            return False
        if PIL_AVAILABLE:
            for path in FONT_PATHS:
                if os.path.exists(path):
                    self.font_path = path
                    break
            if self.font_path:
                print(f"Using font: {self.font_path}")
            else:
                print("No CJK font found. Chinese text may show as ???")
        else:
            print("Pillow not installed. Chinese text may show as ???")
        self.prepared = True
        return True

    def handle_text(self, text, is_final=False):
        matched, value, new_last_sent = map_command(text, self.last_sent)
        if matched:
            self.last_sent = new_last_sent
            with self.state_lock:
                self.last_command = value
                self.last_command_ts = time.time()
                self.latest_partial = ""
            return True
        if text:
            with self.state_lock:
                self.latest_partial = text
        return False

    def update_audio_level(self, level):
        with self.state_lock:
            self.latest_level = level

    def get_font(self, size):
        if size not in self.font_cache:
            self.font_cache[size] = ImageFont.truetype(self.font_path, size=size)
        return self.font_cache[size]

    def render(self, text, subtext="", width=640, height=480):
        image = np.zeros((height, width, 3), dtype=np.uint8)
        if PIL_AVAILABLE and self.font_path:
            img = Image.new("RGB", (width, height), (0, 0, 0))
            draw = ImageDraw.Draw(img)
            if len(text) <= 2:
                font_size = 200
            else:
                font_size = 64
            font = self.get_font(font_size)
            bbox = draw.textbbox((0, 0), text, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            x = max(0, (width - tw) // 2)
            y = max(0, (height - th) // 2)
            draw.text((x, y), text, font=font, fill=(0, 255, 0))
            if subtext:
                small_font = self.get_font(28)
                draw.text((10, height - 40), subtext, font=small_font, fill=(200, 200, 200))
            return np.array(img)[:, :, ::-1]
        font = cv2.FONT_HERSHEY_SIMPLEX
        if len(text) <= 2:
            font_scale = 6.0
            thickness = 12
        else:
            font_scale = 2.0
            thickness = 4
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        x = max(0, (width - tw) // 2)
        y = max(th + 10, (height + th) // 2)
        cv2.putText(image, text, (x, y), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
        if subtext:
            cv2.putText(image, subtext, (10, height - 20), font, 0.8, (200, 200, 200), 2, cv2.LINE_AA)
        return image

    def run_loop(self):
        if not self.prepare():
            return False
        print("listening... press q to quit")
        try:
            while True:
                with self.state_lock:
                    partial = self.latest_partial
                    level = self.latest_level
                    last_command = self.last_command
                    last_command_ts = self.last_command_ts
                now = time.time()
                if last_command and (now - last_command_ts) <= self.hold_seconds:
                    display_text = last_command
                elif partial:
                    display_text = partial
                else:
                    display_text = "Listening"
                cv2.imshow(self.window_title, self.render(display_text, subtext=f"Audio level: {level}"))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                try:
                    if cv2.getWindowProperty(self.window_title, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except Exception:
                    pass
        finally:
            cv2.destroyAllWindows()
        return True

class AudioAnalyzer:
    def __init__(self, model_path, sample_rate, on_result=None, on_partial=None):
        self.model_path = model_path
        self.sample_rate = sample_rate
        self.on_result = on_result
        self.on_partial = on_partial
        self.queue = queue.Queue(maxsize=200)
        self.running = False
        self.worker = None
        self.recognizer = None
        self._last_partial = ""
        self._last_partial_ts = 0.0

    def start(self):
        if self.running:
            return True
        try:
            from vosk import Model, KaldiRecognizer
        except Exception as e:
            print(f"导入 vosk 失败: {e}")
            return False
        if not os.path.isdir(self.model_path):
            print(f"模型路径不存在: {self.model_path}")
            return False
        try:
            model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(model, self.sample_rate)
        except Exception as e:
            print(f"加载模型失败: {e}")
            return False
        self.running = True
        self.worker = threading.Thread(target=self._run, name="AudioAnalyzer", daemon=True)
        self.worker.start()
        print(f"音频分析已启动: {self.model_path} @ {self.sample_rate}Hz")
        return True

    def stop(self):
        if not self.running:
            return
        self.running = False
        self.worker = None
        self.recognizer = None

    def enqueue(self, data):
        if not self.running:
            return
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            pass

    def _run(self):
        while self.running:
            try:
                data = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get("text", "").strip()
                    if text and self.on_result:
                        self.on_result(text)
                else:
                    result = json.loads(self.recognizer.PartialResult())
                    partial = result.get("partial", "").strip()
                    if partial and self._should_emit_partial(partial) and self.on_partial:
                        self.on_partial(partial)
            except Exception:
                continue

    def _should_emit_partial(self, partial):
        now = time.time()
        if partial == self._last_partial and (now - self._last_partial_ts) < 0.5:
            return False
        self._last_partial = partial
        self._last_partial_ts = now
        return True

def resolve_default_model_path():
    env_path = os.environ.get("VOSK_MODEL_PATH")
    if env_path:
        return env_path
    candidates = [
        os.path.join(os.path.dirname(__file__), "model", "vosk-model-small-cn-0.22"),
        os.path.join(os.getcwd(), "model", "vosk-model-small-cn-0.22"),
    ]
    for path in candidates:
        if os.path.isdir(path):
            return path
    return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="经典蓝牙 IMU+音频 数据接收器")
    parser.add_argument("address", help="设备 MAC 地址")
    parser.add_argument("port", nargs="?", type=int, help="RFCOMM 端口 (可选)")
    parser.add_argument("--model-path", default=resolve_default_model_path(), help="Vosk 模型路径")
    parser.add_argument("--no-analyze", action="store_true", help="禁用音频分析")
    parser.add_argument("--no-ui", action="store_true", help="禁用实时识别窗口")
    args = parser.parse_args()

    enable_analysis = not args.no_analyze and args.model_path is not None
    if not args.no_analyze and args.model_path is None:
        print("未找到模型路径，音频分析已关闭。可通过 --model-path 或 VOSK_MODEL_PATH 指定模型。")

    display = None if args.no_ui else DisplayManager()
    if display and not display.prepare():
        display = None

    receiver = BluetoothClassicReceiver(
        enable_analysis=enable_analysis,
        model_path=args.model_path,
        display=display,
    )

    if display:
        connect_thread = threading.Thread(
            target=receiver.connect,
            args=(args.address, args.port),
            daemon=True
        )
        connect_thread.start()
        try:
            display.run_loop()
        finally:
            receiver.stop()
    else:
        receiver.connect(args.address, args.port)
