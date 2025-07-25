import tkinter as tk
from enum import Enum
from collections import deque
import time
import cv2
import numpy as np
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
import subprocess
import os

# ... (ButtonType, Direction, Request 類別維持不變) ...
class ButtonType(Enum):
    UP = 1
    DOWN = -1
    INTERNAL = 0

class Direction(Enum):
    UP = 1
    DOWN = -1
    IDLE = 0

class Request:
    def __init__(self, floor, button_type):
        self.floor = floor
        self.button_type = button_type
        self.timestamp = time.time()

class ElevatorControlSim:
    def __init__(self, master):
        self.master = master
        master.title("Elevator Operation Preview Application")
        self.cap = cv2.VideoCapture(0)
        # 調整MOG2參數以提高小物體檢測的準確性
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=300,         # 減少歷史幀數 (從500減到300)
            varThreshold=50,     # 提高變化檢測閾值 (從16提高到50，降低敏感度)
            detectShadows=True
        )
        self.penetration_area = 0 
        self.total_area = 0
        self.penetration_ratio = 0 
        self.prev_mask = None
        self.baseline_established = False
        self.stabilization_frames = 0
        self.display_width = 240
        self.display_height = 180
        
        # 簡化的學習率控制 - 專為演示設計
        self.initial_learning_rate = 0.01  # 初始學習率：較快建立背景
        self.stable_learning_rate = 0.0005  # 穩定後學習率：很低，防止靜態物體被學習為背景
        self.current_learning_rate = self.initial_learning_rate
        
        # 辨識區域設定 (x, y, width, height)
        # 將左邊邊界往右調整 20% 的畫面寬度
        self.detection_roi = None  # 將在影像處理時動態設定
        
        # --- MODIFICATION START: Serial Communication Setup ---
        self.arduino_serial = "/dev/cu.usbserial-120"
        self.setup_serial()
        # --- MODIFICATION END ---
        
        self.canvas = tk.Canvas(master, width=300, height=600, bg="#808080")
        self.canvas.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)

        # 繪製滿版電梯井背景（深灰色主題）
        self.canvas.create_rectangle(0, 0, 300, 600, fill="#808080", outline="#666666", width=2)
        
        # 繪製磚頭風格背景
        brick_width = 30
        brick_height = 15
        for y in range(0, 600, brick_height):
            for x in range(0, 300, brick_width):
                # 交錯排列磚頭
                offset = brick_width // 2 if (y // brick_height) % 2 == 1 else 0
                brick_x = x + offset
                if brick_x < 300:
                    self.canvas.create_rectangle(
                        brick_x, y, brick_x + brick_width, y + brick_height,
                        fill="#707070", outline="#606060", width=1
                    )
        
        # 繪製樓層
        self.floor_positions = {1: 500, 2: 300, 3: 100}
        for floor, y in self.floor_positions.items():
            # 樓層地板（深色）
            self.canvas.create_line(0, y, 300, y, fill="#222222", width=3)
            # 樓層門框（深色）
            self.canvas.create_rectangle(50, y-5, 250, y+5, fill="#333333", outline="#222222", width=1)
            # 樓層標示（深色）
            self.canvas.create_text(270, y - 15, text=f"{floor}F", font=("Arial", 12, "bold"), fill="#111111")

        # 電梯車廂設計（深色主題）
        self.elevator_width = 80
        self.elevator_height = 80  # 增加高度
        initial_x = 110  # 保持在中央位置
        initial_y = self.floor_positions[1] - self.elevator_height
        
        # 電梯車廂主體（深色金屬質感）
        self.elevator_rect = self.canvas.create_rectangle(
            initial_x, initial_y, initial_x + self.elevator_width, initial_y + self.elevator_height, 
            fill="#404040", outline="#666666", width=2
        )
        
        # 電梯門（深色）
        self.elevator_door_left = self.canvas.create_rectangle(
            initial_x + 5, initial_y + 5, initial_x + 35, initial_y + self.elevator_height - 5,
            fill="#555555", outline="#777777", width=1
        )
        self.elevator_door_right = self.canvas.create_rectangle(
            initial_x + 45, initial_y + 5, initial_x + 75, initial_y + self.elevator_height - 5,
            fill="#555555", outline="#777777", width=1
        )
        self.current_floor = 1
        self.target_floor = None
        self.direction = Direction.IDLE
        self.is_moving_flag = False

        self.internal_requests = []
        self.external_requests = []
        self.pending_external_requests = deque()

        self.full_load = False        
        self.manual_emergency = False   
        self.auto_emergency = False
        self.auto_emergency_active = False  # 追蹤自動Express模式是否正在執行     

        self.control_frame = tk.Frame(master)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=True, padx=5)

        self.camera_frame = tk.Frame(self.control_frame)
        self.camera_frame.pack(fill=tk.X, pady=5)

        self.camera_label = tk.Label(self.camera_frame)
        self.camera_label.pack(pady=2)

        self.penetration_info_label = tk.Label(self.camera_frame, text=f"物體檢測率: {self.penetration_ratio:.2f}%")
        self.penetration_info_label.pack(pady=2)
        
        self.controls_frame = tk.Frame(self.control_frame)
        self.controls_frame.pack(fill=tk.X, pady=5)
        
        self.reset_bg_button = tk.Button(
            self.controls_frame, text="Reset Background", command=self.reset_background
        )
        self.reset_bg_button.pack(fill=tk.X, padx=5, pady=2)
        
        # 添加閾值調整控制
        threshold_frame = tk.Frame(self.controls_frame)
        threshold_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(threshold_frame, text="物體檢測敏感度:").pack()
        
        # 高強度閾值調整 (物體)
        self.high_threshold_var = tk.IntVar(value=220)
        tk.Label(threshold_frame, text="物體數值:").pack(anchor="w")
        self.high_threshold_scale = tk.Scale(threshold_frame, from_=180, to=250, 
                                           orient=tk.HORIZONTAL, variable=self.high_threshold_var)
        self.high_threshold_scale.pack(fill=tk.X)
        
        # 中等強度閾值調整 (陰影)
        self.medium_threshold_var = tk.IntVar(value=150)
        tk.Label(threshold_frame, text="陰影數值:").pack(anchor="w")
        self.medium_threshold_scale = tk.Scale(threshold_frame, from_=100, to=200,
                                             orient=tk.HORIZONTAL, variable=self.medium_threshold_var)
        self.medium_threshold_scale.pack(fill=tk.X)
        
        # Express模式觸發閾值調整
        self.emergency_threshold_var = tk.DoubleVar(value=80.0)
        tk.Label(threshold_frame, text="Express模式觸發%:").pack(anchor="w")
        self.emergency_threshold_scale = tk.Scale(threshold_frame, from_=20.0, to=95.0, resolution=5.0,
                                                 orient=tk.HORIZONTAL, variable=self.emergency_threshold_var)
        self.emergency_threshold_scale.pack(fill=tk.X)


        self.full_load_var = tk.BooleanVar(value=False)
        self.full_load_check = tk.Checkbutton(
            self.controls_frame, text="Emergency Button", variable=self.full_load_var, command=self.toggle_full_load
        )
        self.full_load_check.pack(pady=5)

        self.buttons_frame = tk.Frame(self.control_frame)
        self.buttons_frame.pack(fill=tk.X, pady=5)
        
        self.internal_frame = tk.Frame(self.buttons_frame)
        self.internal_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        tk.Label(self.internal_frame, text="Inside Request").pack(pady=2)
        self.btn_internal_floor3 = tk.Button(
            self.internal_frame, text="3", command=lambda: self.add_request(3, ButtonType.INTERNAL)
        )
        self.btn_internal_floor3.pack(fill=tk.X, padx=2, pady=1)
        self.btn_internal_floor2 = tk.Button(
            self.internal_frame, text="2", command=lambda: self.add_request(2, ButtonType.INTERNAL)
        )
        self.btn_internal_floor2.pack(fill=tk.X, padx=2, pady=1)
        self.btn_internal_floor1 = tk.Button(
            self.internal_frame, text="1", command=lambda: self.add_request(1, ButtonType.INTERNAL)
        )
        self.btn_internal_floor1.pack(fill=tk.X, padx=2, pady=1)

        self.external_frame = tk.Frame(self.buttons_frame)
        self.external_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        
        tk.Label(self.external_frame, text="Outside Request").pack(pady=2)
        self.btn_ext_3_down = tk.Button(
            self.external_frame, text="3↓", width=4, command=lambda: self.add_request(3, ButtonType.DOWN)
        )
        self.btn_ext_3_down.pack(fill=tk.X, padx=2, pady=1)
        self.btn_ext_2_up = tk.Button(
            self.external_frame, text="2↑", width=4, command=lambda: self.add_request(2, ButtonType.UP)
        )
        self.btn_ext_2_up.pack(fill=tk.X, padx=2, pady=1)
        self.btn_ext_2_down = tk.Button(
            self.external_frame, text="2↓", width=4, command=lambda: self.add_request(2, ButtonType.DOWN)
        )
        self.btn_ext_2_down.pack(fill=tk.X, padx=2, pady=1)
        self.btn_ext_1_up = tk.Button(
            self.external_frame, text="1↑", width=4, command=lambda: self.add_request(1, ButtonType.UP)
        )
        self.btn_ext_1_up.pack(fill=tk.X, padx=2, pady=1)

        self.info_label = tk.Label(self.control_frame, text="Status：Idle", wraplength=280)
        self.info_label.pack(pady=10)

        self.master.after(100, self.simulation_loop)
        self.master.after(100, self.update_penetration_detection)
        self.master.after(100, self.start_arduino_button_check)  # 啟動 Arduino 按鈕檢查
    
    # --- FIX 1: MODIFIED setup_serial ---
    def setup_serial(self):
        """自動尋找並連接到 Arduino，如果失敗則提供除錯資訊。"""
        print("Finding Arduino...")
        ports = serial.tools.list_ports.comports()
        arduino_port = "/dev/cu.usbserial-120"
        
        if not ports:
            print("Error：Can't find any serial.")
            return

        for port in ports:
            # 在 MacBook 上，Arduino 通常顯示為 'usbmodem'
            # 在 Windows 上，可能是 'Arduino' in port.description
            # 某些仿製版可能沒有可識別的名稱
            if 'usbmodem' in port.device or 'Arduino' in str(port.description):
                arduino_port = port.device
                print(f"Found Arduino in: {arduino_port}")
                break
        
        if arduino_port:
            try:
                self.arduino_serial = serial.Serial(arduino_port, 9600, timeout=1)
                time.sleep(2) # 等待 Arduino 重啟
                print("Connect Success。")
            except serial.SerialException as e:
                print(f"Failed to connect Arduino: {e}")
                self.arduino_serial = None
        else:
            # 如果找不到自動識別的Arduino，嘗試使用預設埠
            try:
                self.arduino_serial = serial.Serial("/dev/cu.usbserial-120", 9600, timeout=1)
                time.sleep(2) # 等待 Arduino 重啟
                print("Connect Success using default port。")
            except serial.SerialException as e:
                print(f"Failed to connect Arduino: {e}")
                print("Error：找不到可自動識別的 Arduino 模組。")
                print("請檢查連接，並從以下可用埠列表中找到您的 Arduino：")
                for port in ports:
                    print(f" - {port.device}: {port.description}")
                print("\nPlease modify the serial in the 45 & 161 in the code")
                self.arduino_serial = None

    # --- FIX 2: MODIFIED send_to_arduino with better error handling ---
    # 在 main_elevator_simulator.py 中修改

    def send_to_arduino(self, status, floor, direction):
        """格式化並發送狀態給 Arduino，包含更安全的檢查。"""
        if isinstance(self.arduino_serial, serial.Serial) and self.arduino_serial.is_open:
            # 獲取目標樓層，使用當前目標樓層（包括中途停靠）
            if hasattr(self, 'target_floor') and self.target_floor:
                target_floor = self.target_floor
            else:
                target_floor = floor
            command = f"STATUS:{status};FLOOR:{floor};DIR:{direction.name};TARGET:{target_floor};\n"
            try:
                # --- 在這裡加入一行 print 語句 ---
                print(f"--> [PYTHON SENDS] Sending command: {command.strip()}")
                self.arduino_serial.write(command.encode('utf-8'))
            except serial.SerialException as e:
                print(f"Command send error: {e}")
                self.arduino_serial.close()
                self.arduino_serial = None

    def get_current_module_status(self):
        """根據電梯狀態決定要傳送給模組的狀態字串。"""
        if self.manual_emergency:
            return "EMERGENCY" # 手動緊急優先級最高
        elif self.auto_emergency:
            return "FULL" # 自動偵測滿載
        else:
            return "NORMAL"
    
    def reset_background(self):
        # 使用調整後的MOG2參數重新建立背景減法器
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=300,         # 減少歷史幀數
            varThreshold=50,     # 提高變化檢測閾值，降低敏感度
            detectShadows=True
        )
        self.baseline_established = False
        self.stabilization_frames = 0
        # 重置學習率狀態
        self.current_learning_rate = self.initial_learning_rate
        # 重置閾值為默認值
        self.high_threshold_var.set(220)
        self.medium_threshold_var.set(150)
        self.emergency_threshold_var.set(80.0)
        print("🔄 背景重置完成 - 使用智能強度檢測重新建立背景模型")
    



    def update_emergency_mode(self):
        prev_full_load = self.full_load
        self.full_load = self.manual_emergency or self.auto_emergency
        if prev_full_load != self.full_load:
            self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)

    def toggle_full_load(self):
        prev_emergency = self.full_load
        self.manual_emergency = self.full_load_var.get()
        self.update_emergency_mode()
        
        if self.manual_emergency:
            print("🚨 手動：電梯已進入Express模式 - 不回應外部請求，正常處理內部請求")
        else:
            print("✅ 手動：電梯已解除Express模式")
            if prev_emergency:
                print("電梯從Express模式恢復正常運作")
                # 清空暫存的外部請求，電梯停在當前樓層
                if self.pending_external_requests:
                    pending_count = len(self.pending_external_requests)
                    print(f"🗑️  清空 {pending_count} 個暫存的外部請求（Express模式解除後重新開始）")
                    self.pending_external_requests.clear()
                
                # 設置電梯為待機狀態
                self.direction = Direction.IDLE
                print(f"✅ 電梯停留在 {self.current_floor} 樓，等待新的請求")
        
        self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)


    def add_request(self, floor, button_type):
        if floor == self.current_floor and button_type == ButtonType.INTERNAL:
            print(f"忽略當前樓層 {floor} 的內部請求。")
            return
            
        new_request = Request(floor, button_type)
        
        if button_type == ButtonType.INTERNAL:
            # Express模式下仍然正常處理內部請求，不限制數量
            if not any(req.floor == floor for req in self.internal_requests):
                self.internal_requests.append(new_request)
                if self.full_load:
                    print(f"Express模式：接受內部請求 - 樓層 {floor}")
                else:
                    print(f"內部請求：樓層 {floor}")
        else:
            if self.full_load:
                self.pending_external_requests.append(new_request)
                print(f"外部請求：樓層 {floor}（Express模式，暫存）")
            else:
                if not any(req.floor == floor and req.button_type == button_type for req in self.external_requests):
                    self.external_requests.append(new_request)
                    print(f"外部請求：樓層 {floor}，方向：{button_type.name}")
                    
        self.info_label.config(text=f"Status：{self.get_status_text()}")
        if not self.is_moving_flag:
            self.master.after(100, self.process_requests)

    def get_active_requests(self):
        if self.full_load:
            return self.internal_requests
        return self.internal_requests + self.external_requests

    def get_status_text(self):
        if self.full_load:
            if len(self.internal_requests) == 0:
                return f"🚨 Express模式：在 {self.current_floor} 樓待命，等待內部請求"
            else:
                # 顯示所有內部請求的樓層
                floors = [str(req.floor) for req in self.internal_requests]
                if self.is_moving_flag:
                    return f"🚨 Express模式：服務內部請求 {','.join(floors)} 樓"
                else:
                    return f"🚨 Express模式：準備服務內部請求 {','.join(floors)} 樓"
        else:
            active = self.get_active_requests()
            reqs = "無請求" if not active else ", ".join(f"{req.floor}({req.button_type.name})" for req in active)
            overall = "啟動" if self.full_load else "解除"
            manual = "啟動" if self.manual_emergency else "解除"
            auto = "啟動" if self.auto_emergency else "解除"
            return (f"{'Moving' if self.is_moving_flag else 'IDLE'}（{self.current_floor} 樓），請求：{reqs}；"
                    f" Emergency Mode(All:{overall}, Manual:{manual}, Auto:{auto})")

    def process_requests(self):
        active_requests = self.get_active_requests()
        if not active_requests:
            self.info_label.config(text=f"Status：waiting in {self.current_floor}F ")
            self.direction = Direction.IDLE
            self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
            return
            
        next_stop = self.get_next_stop()
        if next_stop is not None:
            self.target_floor = next_stop
            if self.target_floor > self.current_floor:
                self.direction = Direction.UP
            elif self.target_floor < self.current_floor:
                self.direction = Direction.DOWN
            else:
                self.direction = Direction.IDLE
            self.info_label.config(text=f"Moving {self.direction.name} to {self.target_floor} F")
            self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
            self.animate_movement(self.current_floor, self.target_floor, frames=60)
        else:
            self.info_label.config(text="No next stop")

    def get_next_stop(self):
        active_requests = self.get_active_requests()
        if not active_requests:
            return None
        
        if self.full_load:
            return self.get_next_internal_stop(active_requests)
        
        if self.direction == Direction.UP:
            upper_stops = [req.floor for req in active_requests if req.floor > self.current_floor]
            if upper_stops:
                return min(upper_stops)
        elif self.direction == Direction.DOWN:
            lower_stops = [req.floor for req in active_requests if req.floor < self.current_floor]
            if lower_stops:
                return max(lower_stops)
        nearest_stop = min(active_requests, key=lambda req: abs(req.floor - self.current_floor)).floor
        return nearest_stop
    
    def get_next_internal_stop(self, internal_requests):
        if not internal_requests:
            return None
        # 在Express模式下也使用正常的電梯邏輯選擇最近的樓層
        nearest_stop = min(internal_requests, key=lambda req: abs(req.floor - self.current_floor)).floor
        print(f"Express模式：moving to {nearest_stop} F")
        return nearest_stop

    def remove_completed_requests(self):
        self.internal_requests = [req for req in self.internal_requests if req.floor != self.current_floor]
        if not self.full_load:
            self.external_requests = [req for req in self.external_requests if req.floor != self.current_floor]

    def animate_movement(self, start_floor, end_floor, frames):
        self.is_moving_flag = True
        self.anim_start_floor = start_floor
        start_y = self.floor_positions[start_floor] - self.elevator_height
        end_y = self.floor_positions[end_floor] - self.elevator_height
        self.animation_frame = 0
        
        # 設定移動時間為 3 秒，每 30ms 更新一次 (更快更流暢)
        frame_interval = 30  # 毫秒 (從50ms減少到30ms)
        movement_time = 3.0  # 秒 (從7.5秒減少到3秒)
        self.total_frames = int((movement_time * 1000) / frame_interval)
        
        # 記錄開始時間用於除錯
        self.movement_start_time = time.time()
        print(f"電梯開始移動：從 {start_floor} 樓到 {end_floor} 樓，預計時間 {movement_time} 秒")
        
        # 保存最終目標樓層，不讓中途停靠改變它
        self.final_target_floor = end_floor
        self.target_floor = end_floor
        self.dy = (end_y - start_y) / self.total_frames

        def step():
            if self.animation_frame < self.total_frames:
                # 調整中途停靠檢查頻率 (從每20幀改為每15幀，配合更快的更新)
                if self.animation_frame % 15 == 0:
                    active = self.get_active_requests()
                    coords = self.canvas.coords(self.elevator_rect)
                    current_y = coords[1]
                    
                    # Express模式下的特殊2樓停靠邏輯
                    if self.full_load and self.direction == Direction.UP:
                        # 檢查是否有2樓的請求
                        floor_2_requests = [req for req in active if req.floor == 2]
                        if floor_2_requests:
                            # 檢查是否正從1樓往上移動且還沒經過2樓
                            floor_2_y = self.floor_positions[2] - self.elevator_height
                            # 如果電梯還在2樓以下且目標樓層高於2樓
                            if current_y > floor_2_y and self.target_floor > 2:
                                print(f"Express模式特殊規則：在1-2樓之間檢測到2樓請求，停在2樓")
                                self.target_floor = 2
                                # 立即發送狀態更新
                                self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
                                # 調整移動軌跡到2樓
                                new_end_y = floor_2_y
                                remaining_frames = self.total_frames - self.animation_frame
                                if remaining_frames > 0:
                                    self.dy = (new_end_y - current_y) / remaining_frames
                    
                    # 正常模式下的中途停靠邏輯
                    elif not self.full_load:
                        if self.direction == Direction.UP:
                            possible = []
                            for req in active:
                                if req.button_type in (ButtonType.UP, ButtonType.INTERNAL):
                                    if (self.anim_start_floor < req.floor < self.target_floor):
                                        stop_y = self.floor_positions[req.floor] - self.elevator_height
                                        if current_y > stop_y:
                                            possible.append(req.floor)
                            if possible:
                                new_target = min(possible)
                                if new_target < self.target_floor:
                                    print(f"中途請求：改為先停 {new_target} 樓")
                                    # 只改變當前目標，不改變最終目標
                                    self.target_floor = new_target
                                    # 立即發送狀態更新
                                    self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
                                    # 保持原有移動速度，不延長總時間
                                    new_end_y = self.floor_positions[new_target] - self.elevator_height
                                    remaining_frames = self.total_frames - self.animation_frame
                                    self.dy = (new_end_y - current_y) / remaining_frames
                        elif self.direction == Direction.DOWN:
                            possible = []
                            for req in active:
                                if req.button_type in (ButtonType.DOWN, ButtonType.INTERNAL):
                                    if (self.anim_start_floor > req.floor > self.target_floor):
                                        stop_y = self.floor_positions[req.floor] - self.elevator_height
                                        if current_y < stop_y:
                                            possible.append(req.floor)
                            if possible:
                                new_target = max(possible)
                                if new_target > self.target_floor:
                                    print(f"中途請求：改為先停 {new_target} 樓")
                                    self.target_floor = new_target
                                    # 立即發送狀態更新
                                    self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
                                    # 保持原有移動速度，不延長總時間
                                    new_end_y = self.floor_positions[new_target] - self.elevator_height
                                    remaining_frames = self.total_frames - self.animation_frame
                                    self.dy = (new_end_y - current_y) / remaining_frames

                self.canvas.move(self.elevator_rect, 0, self.dy)
                self.canvas.move(self.elevator_door_left, 0, self.dy)
                self.canvas.move(self.elevator_door_right, 0, self.dy)
                self.animation_frame += 1
                
                # 計算當前樓層位置
                coords = self.canvas.coords(self.elevator_rect)
                current_y = coords[1]
                
                # 根據Y座標計算當前樓層
                if self.direction == Direction.UP:
                    # 上升時，根據Y座標判斷當前樓層
                    if current_y <= self.floor_positions[2] - self.elevator_height:
                        current_floor = 1
                    elif current_y <= self.floor_positions[3] - self.elevator_height:
                        current_floor = 2
                    else:
                        current_floor = 3
                elif self.direction == Direction.DOWN:
                    # 下降時，根據Y座標判斷當前樓層
                    if current_y >= self.floor_positions[2] - self.elevator_height:
                        current_floor = 3
                    elif current_y >= self.floor_positions[1] - self.elevator_height:
                        current_floor = 2
                    else:
                        current_floor = 1
                else:
                    current_floor = self.current_floor
                
                # 更新當前樓層並發送狀態
                if current_floor != self.current_floor:
                    self.current_floor = current_floor
                    self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
                
                # 每33幀（約1秒）發送一次狀態更新 (配合30ms間隔: 30ms * 33 = 990ms ≈ 1秒)
                if self.animation_frame % 33 == 0:
                    self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
                
                self.master.after(frame_interval, step)
            else:
                coords = self.canvas.coords(self.elevator_rect)
                if self.target_floor is not None:
                    final_y = self.floor_positions[self.target_floor] - self.elevator_height
                    self.canvas.coords(self.elevator_rect, coords[0], final_y,
                                       coords[0] + self.elevator_width, final_y + self.elevator_height)
                    
                    # 同時定位電梯門
                    door_left_coords = self.canvas.coords(self.elevator_door_left)
                    door_right_coords = self.canvas.coords(self.elevator_door_right)
                    self.canvas.coords(self.elevator_door_left, door_left_coords[0], final_y + 5,
                                       door_left_coords[2], final_y + self.elevator_height - 5)
                    self.canvas.coords(self.elevator_door_right, door_right_coords[0], final_y + 5,
                                       door_right_coords[2], final_y + self.elevator_height - 5)
                    self.current_floor = self.target_floor
                self.is_moving_flag = False
                self.remove_completed_requests()
                
                # 計算實際移動時間
                actual_time = time.time() - self.movement_start_time
                print(f"電梯已到達 {self.current_floor} 樓，實際移動時間：{actual_time:.2f} 秒")
                
                # 播放樓層音效
                self.play_floor_sound(self.current_floor)
                
                self.direction = Direction.IDLE
                self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)

                # 如果是自動Express模式，在抵達目標樓層後重置active狀態，但保持auto_emergency讓檢測邏輯處理
                if self.auto_emergency_active:
                    print("🎯 Express模式：已抵達目標樓層")
                    self.auto_emergency_active = False
                    print("ℹ️  Express模式狀態將由突破量檢測自動管理")
                
                if self.full_load:
                    # 在Express模式下，完成當前樓層的請求，但如果還有其他樓層的請求則繼續服務
                    self.remove_completed_requests()
                    remaining_requests = len(self.internal_requests)
                    if remaining_requests > 0:
                        print(f"🚨 Express模式：已到達 {self.current_floor} 樓，還有 {remaining_requests} 個內部請求待處理")
                        self.info_label.config(text=f"Express模式：已到達 {self.current_floor} 樓，繼續處理剩餘請求")
                        self.master.after(500, self.process_requests)
                        return
                    else:
                        print("🚨 Express模式：所有內部請求已完成，在此樓層待命")
                        self.info_label.config(text=f"Express模式：待命中 - 在 {self.current_floor} 樓")
                        return 
                
                self.info_label.config(text=f"已到 {self.current_floor} 樓。{self.get_status_text()}")
                
                # 移除自動處理暫存請求的邏輯，解除Express模式後停在當前樓層
                if not self.full_load and self.pending_external_requests:
                    pending_count = len(self.pending_external_requests)
                    print(f"🗑️  移動完成後清空 {pending_count} 個暫存的外部請求")
                    self.pending_external_requests.clear()
                
                self.master.after(500, self.process_requests)
        step()

    def update_penetration_detection(self):
        ret, frame = self.cap.read()
        if ret:
            # ... (影像處理邏輯不變，除了 send_to_arduino 的呼叫) ...
            if not self.baseline_established:
                self.stabilization_frames += 1
                if self.stabilization_frames > 10:
                    self.baseline_established = True
                    # 背景基準建立完成後，切換到低學習率
                    self.current_learning_rate = self.stable_learning_rate
                    print("🎯 背景基準已建立完成 - 切換到穩定學習率")
                # 初始化階段建立背景模型
                self.background_subtractor.apply(frame, learningRate=self.initial_learning_rate)
                
                display_frame = frame.copy()
                cv2.putText(display_frame, f"建立背景基準中 ({self.stabilization_frames}/10)...", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                display_frame = cv2.resize(display_frame, (self.display_width, self.display_height))
                
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(rgb_frame)
                photo = ImageTk.PhotoImage(image)
                self.camera_label.config(image=photo)
                self.camera_label.image = photo
                
                self.master.after(100, self.update_penetration_detection)
                return
            
            # 設定辨識區域 - 調整左右邊界
            height, width = frame.shape[:2]
            left_offset = int(width * 0.1)  # 左邊邊界往右調整 10%（比之前往左調整）
            right_offset = int(width * 0.03)  # 右邊邊界往左調整 3%（比之前往右調整）
            self.detection_roi = (left_offset, 0, width - left_offset - right_offset, height)
            
            # 只處理辨識區域內的影像
            x, y, w, h = self.detection_roi
            roi_frame = frame[y:y+h, x:x+w]
            
            self.total_area = w * h  # 只計算辨識區域的面積
            # 使用調整後的學習率防止靜態物體被誤學習為背景
            fg_mask = self.background_subtractor.apply(roi_frame, learningRate=self.current_learning_rate)
            
            # 智能強度檢測 - 區分真正物體和陰影干擾
            # 先進行輕微模糊去除噪音
            fg_mask_blurred = cv2.GaussianBlur(fg_mask, (3, 3), 0)
            
            # 分析強度分佈，創建多層次檢測
            # 高強度閾值：檢測真正的物體（深紅色區域）
            high_intensity_threshold = self.high_threshold_var.get()  # 使用滑動條的值
            medium_intensity_threshold = self.medium_threshold_var.get()  # 使用滑動條的值
            
            # 創建不同強度的遮罩
            _, high_intensity_mask = cv2.threshold(fg_mask_blurred, high_intensity_threshold, 255, cv2.THRESH_BINARY)
            _, medium_intensity_mask = cv2.threshold(fg_mask_blurred, medium_intensity_threshold, 255, cv2.THRESH_BINARY)
            
            # 形態學操作 - 針對高強度物體優化
            kernel_small = np.ones((2, 2), np.uint8)
            kernel_medium = np.ones((3, 3), np.uint8)
            
            # 高強度物體處理（主要計算目標）
            high_intensity_mask = cv2.morphologyEx(high_intensity_mask, cv2.MORPH_OPEN, kernel_small)
            high_intensity_mask = cv2.morphologyEx(high_intensity_mask, cv2.MORPH_CLOSE, kernel_medium)
            
            # 中等強度處理（僅用於視覺化）
            medium_intensity_mask = cv2.morphologyEx(medium_intensity_mask, cv2.MORPH_OPEN, kernel_small)
            
            # 主要突破量計算：只使用高強度區域
            self.penetration_area = cv2.countNonZero(high_intensity_mask)
            self.penetration_ratio = (self.penetration_area / self.total_area) * 100
            
            # 計算額外統計信息
            medium_area = cv2.countNonZero(medium_intensity_mask)
            shadow_area = medium_area - self.penetration_area  # 陰影區域
            
            # 使用高強度遮罩作為主要檢測結果
            fg_mask = high_intensity_mask
            self.penetration_info_label.config(text=f"物體檢測率: {self.penetration_ratio:.2f}%")
            
            # 創建多層次視覺化 - 區分物體和陰影
            full_high_mask = np.zeros((height, width), dtype=np.uint8)
            full_medium_mask = np.zeros((height, width), dtype=np.uint8)
            
            # 將不同強度遮罩放回原始影像位置
            full_high_mask[y:y+h, x:x+w] = high_intensity_mask
            full_medium_mask[y:y+h, x:x+w] = medium_intensity_mask
            
            # 創建彩色遮罩
            visualization = frame.copy()
            
            # 高強度物體：深紅色（主要檢測目標）
            high_mask_colored = cv2.cvtColor(full_high_mask, cv2.COLOR_GRAY2BGR)
            high_mask_colored[np.where((high_mask_colored == [255, 255, 255]).all(axis=2))] = [0, 0, 255]  # 深紅色
            
            # 中等強度陰影：淺黃色（參考用）
            medium_mask_colored = cv2.cvtColor(full_medium_mask, cv2.COLOR_GRAY2BGR)
            medium_mask_colored[np.where((medium_mask_colored == [255, 255, 255]).all(axis=2))] = [0, 255, 255]  # 淺黃色
            
            # 先疊加陰影（較淺），再疊加物體（較深）
            alpha_shadow = 0.3
            alpha_object = 0.6
            
            # 疊加陰影區域
            shadow_only_mask = full_medium_mask - full_high_mask  # 只有陰影的區域
            shadow_only_colored = cv2.cvtColor(shadow_only_mask, cv2.COLOR_GRAY2BGR)
            shadow_only_colored[np.where((shadow_only_colored == [255, 255, 255]).all(axis=2))] = [0, 255, 255]  # 淺黃色
            visualization = cv2.addWeighted(visualization, 1, shadow_only_colored, alpha_shadow, 0)
            
            # 疊加高強度物體區域
            visualization = cv2.addWeighted(visualization, 1, high_mask_colored, alpha_object, 0)
            
            # 在畫面上繪製辨識區域邊界
            cv2.rectangle(visualization, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(visualization, "Detection Area", (x + 5, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(visualization, f"Object Ratio: {self.penetration_ratio:.2f}%", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 獲取當前Express模式觸發閾值
            emergency_threshold = self.emergency_threshold_var.get() / 100.0  # 轉換為比例
            
            # 詳細的多層次檢測信息
            object_text = f"Objects: {self.penetration_area} px (Red)"
            shadow_text = f"Shadows: {shadow_area} px (Yellow)"
            threshold_text = f"Thresholds: Obj={high_intensity_threshold}, Shadow={medium_intensity_threshold}"
            emergency_text = f"Emergency Trigger: {emergency_threshold * 100:.0f}%"
            
            cv2.putText(visualization, object_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)  # 紅色文字
            cv2.putText(visualization, shadow_text, (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)  # 黃色文字
            cv2.putText(visualization, threshold_text, (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)  # 白色文字
            cv2.putText(visualization, emergency_text, (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # 綠色文字
            
            if self.penetration_ratio / 100 >= emergency_threshold:
                cv2.putText(visualization, "⚠️ 物體過多", (10, 150),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if not self.auto_emergency:
                    print(f"偵測到突破量 {self.penetration_ratio:.2f}% 已超過閾值 {emergency_threshold * 100:.0f}%")
                    print("🚨 自動啟動Express模式 (物體過多)")
                    self.auto_emergency = True
                    self.auto_emergency_active = True  # 標記自動Express模式已啟動
                    self.send_to_arduino("FULL", self.current_floor, self.direction)
            else:
                # 修正：簡化自動Express模式解除邏輯，直接檢查auto_emergency狀態
                if self.auto_emergency:
                    prev_emergency = self.full_load
                    print(f"🔄 偵測到突破量 {self.penetration_ratio:.2f}% 已低於閾值 {emergency_threshold * 100:.0f}%")
                    print("✅ 自動解除Express模式")
                    self.auto_emergency = False
                    self.auto_emergency_active = False  # 重置狀態標記
                    self.update_emergency_mode()
                    self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)

                    if prev_emergency and not self.full_load:
                        print("🎯 電梯從Express模式恢復正常運作")
                        # 清空暫存的外部請求，電梯停在當前樓層
                        if self.pending_external_requests:
                            pending_count = len(self.pending_external_requests)
                            print(f"🗑️  清空 {pending_count} 個暫存的外部請求（Express模式解除後重新開始）")
                            self.pending_external_requests.clear()
                        
                        # 設置電梯為待機狀態
                        self.direction = Direction.IDLE
                        print(f"✅ 電梯停留在 {self.current_floor} 樓，等待新的請求")
            
            self.update_emergency_mode()
            
            visualization = cv2.resize(visualization, (self.display_width, self.display_height))
            
            rgb_frame = cv2.cvtColor(visualization, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb_frame)
            photo = ImageTk.PhotoImage(image)
            self.camera_label.config(image=photo)
            self.camera_label.image = photo
            
        self.master.after(100, self.update_penetration_detection)

    def simulation_loop(self):
        self.info_label.config(text=f"Status：{self.get_status_text()}")
        self.send_to_arduino(self.get_current_module_status(), self.current_floor, self.direction)
        self.master.after(1000, self.simulation_loop)
    
    def start_arduino_button_check(self):
        """開始定期檢查 Arduino 按鈕"""
        self.check_arduino_buttons()
        self.master.after(100, self.start_arduino_button_check)  # 每 100ms 檢查一次
    
    def check_arduino_buttons(self):
        """檢查 Arduino 傳來的按鈕訊號"""
        if isinstance(self.arduino_serial, serial.Serial) and self.arduino_serial.is_open:
            try:
                if self.arduino_serial.in_waiting > 0:
                    line = self.arduino_serial.readline().decode('utf-8').strip()
                    print(f"<-- [ARDUINO SENDS] {line}")
                    
                    if line.startswith("BUTTON:"):
                        self.handle_arduino_button(line)
                    elif line.startswith("PLAY_SOUND:"):
                        # 處理音效播放指令
                        sound_file = line.split(":")[1]
                        self.play_sound(sound_file)
            except serial.SerialException as e:
                print(f"Arduino 通訊錯誤: {e}")
    
    def handle_arduino_button(self, button_signal):
        """處理 Arduino 按鈕訊號"""
        if button_signal == "BUTTON:1":
            print("Arduino 一樓按鈕被按下")
            self.add_request(1, ButtonType.INTERNAL)
        elif button_signal == "BUTTON:2":
            print("Arduino 二樓按鈕被按下")
            self.add_request(2, ButtonType.INTERNAL)
        elif button_signal == "BUTTON:3":
            print("Arduino 三樓按鈕被按下")
            self.add_request(3, ButtonType.INTERNAL)
        elif button_signal == "BUTTON:EMERGENCY_ON":
            print("Arduino 緊急按鈕被按下 - 進入Emergency模式")
            self.full_load_var.set(True)
            self.toggle_full_load()
        elif button_signal == "BUTTON:EMERGENCY_OFF":
            print("Arduino 緊急按鈕被按下 - 解除Emergency模式")
            self.full_load_var.set(False)
            self.toggle_full_load()
        elif button_signal.startswith("PLAY_SOUND:"):
            # 處理音效播放指令
            sound_file = button_signal.split(":")[1]
            self.play_sound(sound_file)
    
    def play_floor_sound(self, floor):
        """播放樓層音效"""
        sound_file = f"{floor}f.mp3"
        self.play_sound(sound_file)
    
    def play_sound(self, sound_file):
        """播放音效檔案"""
        try:
            # 檢查音效檔案是否存在（在 Code 資料夾下）
            sound_path = os.path.join(os.path.dirname(__file__), sound_file)
            print(f"嘗試播放音效: {sound_file}")
            print(f"音效檔案路徑: {sound_path}")
            print(f"檔案是否存在: {os.path.exists(sound_path)}")
            
            if os.path.exists(sound_path):
                # 使用 afplay 播放音效 (macOS)
                subprocess.Popen(["afplay", sound_path])
                print(f"✅ 成功播放音效: {sound_file}")
            else:
                print(f"❌ 音效檔案不存在: {sound_path}")
        except Exception as e:
            print(f"❌ 播放音效失敗: {e}")

    def on_closing(self):
        if isinstance(self.arduino_serial, serial.Serial) and self.arduino_serial.is_open:
            print("關閉 Arduino 連接...")
            self.arduino_serial.close()
        self.cap.release()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    sim = ElevatorControlSim(root)
    root.protocol("WM_DELETE_WINDOW", sim.on_closing)
    root.mainloop()