# FILE: /Joint stiffness/Single_Motor_gui.py
import sys
import threading
import time
from dynamixel_sdk import *
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QGridLayout, QFrame, QDialog, QCheckBox, QComboBox
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
import matplotlib.pyplot as plt

# ─────────────────────────────────────────── SETTINGS
PORT_NAME        = "COM4"
BAUDRATE         = 57600
PROTOCOL_VERSION = 2.0
DXL_ID           = 13

TICKS_PER_REV = 4096
STEP_REV      = 0.02
MAX_REV       =  10.0
MIN_REV       = -10.0

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT  = 126
ADDR_PRESENT_VOLTAGE  = 144
ADDR_PRESENT_TEMP     = 146
LEN_GOAL_POSITION     = 4
LEN_PRESENT_POSITION  = 4

# ─────────────────────────────────────────── DYNAMIXEL INIT
portHandler   = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("Failed to open port"); sys.exit(1)
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate"); sys.exit(1)

def init_motor(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 4)
    time.sleep(0.1)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.05)

init_motor(DXL_ID)

# Read starting position as zero reference
def read_raw(dxl_id):
    val, res, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return val if res == COMM_SUCCESS else 0

zero = read_raw(DXL_ID)

# ─────────────────────────────────────────── SHARED STATE
goal = 0.0
state_lock = threading.Lock()

def pack_goal(rev, zero_offset):
    ticks = int(zero_offset + rev * TICKS_PER_REV) & 0xFFFFFFFF
    return [
        DXL_LOBYTE(DXL_LOWORD(ticks)),
        DXL_HIBYTE(DXL_LOWORD(ticks)),
        DXL_LOBYTE(DXL_HIWORD(ticks)),
        DXL_HIBYTE(DXL_HIWORD(ticks))
    ]

def send_goal(g):
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncWrite.addParam(DXL_ID, pack_goal(g, zero))
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

def read_telemetry(dxl_id, zero_offset):
    pos, r, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    vel, r2,_ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VELOCITY)
    cur, r3,_ = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_CURRENT)
    vol, r4,_ = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VOLTAGE)
    tmp, r5,_ = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_TEMP)

    pos_rev = (pos - zero_offset) / TICKS_PER_REV if r == COMM_SUCCESS else 0.0
    vel_rpm = (vel * 0.229) if r2 == COMM_SUCCESS else 0.0
    if vel_rpm > 2147483647 * 0.229: vel_rpm -= 4294967296 * 0.229
    cur_ma  = ((cur - 65536) * 2.69 if cur > 32767 else cur * 2.69) if r3 == COMM_SUCCESS else 0.0
    vol_v   = (vol * 0.1) if r4 == COMM_SUCCESS else 0.0
    tmp_c   = tmp if r5 == COMM_SUCCESS else 0

    return pos_rev, vel_rpm, cur_ma, vol_v, tmp_c

# ─────────────────────────────────────────── GUI
class GraphDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Parameters to Display")
        self.setLayout(QVBoxLayout())
        
        self.param_selector = QComboBox(self)
        self.param_selector.addItems(["Position", "Velocity", "Current", "Voltage", "Temperature"])
        self.layout().addWidget(self.param_selector)

        self.plot_button = QPushButton("Plot", self)
        self.plot_button.clicked.connect(self.plot_graph)
        self.layout().addWidget(self.plot_button)

    def plot_graph(self):
        selected_param = self.param_selector.currentText()
        # Here you would implement the logic to gather data for the selected parameter
        # For demonstration, we will just plot dummy data
        x = list(range(10))
        y = [i for i in range(10)]  # Replace with actual data based on selected_param

        plt.plot(x, y)
        plt.title(f"{selected_param} over Time")
        plt.xlabel("Time")
        plt.ylabel(selected_param)
        plt.show()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Single Dynamixel Control")
        self.setStyleSheet("background-color: #0d1117; color: #c9d1d9;")
        self.setMinimumWidth(700)

        central = QWidget()
        main_layout = QVBoxLayout()

        # Title
        title = QLabel("DYNAMIXEL CONTROL  ·  ID 13")
        title.setStyleSheet("color: #00d4ff; font-family: Consolas; font-size: 13px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Motor panel
        self.panel = QGroupBox("MOTOR CONTROL")
        self.panel.setStyleSheet("color: #58a6ff; border: 1px solid #58a6ff; border-radius: 6px; margin-top: 10px; font-weight: bold; font-size: 13px;")
        layout = QVBoxLayout()

        self.pos_label = QLabel("Position (rev): --")
        layout.addWidget(self.pos_label)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_left = QPushButton("◀  LEFT")
        self.btn_right = QPushButton("RIGHT  ▶")
        self.btn_zero = QPushButton("⦿ ZERO")
        self.btn_graph = QPushButton("📊 GRAPH")

        for btn in [self.btn_left, self.btn_right, self.btn_zero, self.btn_graph]:
            btn.setStyleSheet("background: #21262d; color: #c9d1d9; border: 1px solid #30363d; border-radius: 4px; padding: 6px; font-family: Consolas; font-size: 11px;")
            btn_layout.addWidget(btn)

        layout.addLayout(btn_layout)
        self.panel.setLayout(layout)
        main_layout.addWidget(self.panel)

        # Status bar
        self.status = QLabel("Ready")
        self.status.setStyleSheet("color: #484f58; font-family: Consolas; font-size: 10px;")
        self.status.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

        # Wire up buttons
        self.btn_left.clicked.connect(lambda: self.move("LEFT"))
        self.btn_right.clicked.connect(lambda: self.move("RIGHT"))
        self.btn_zero.clicked.connect(self.zero)
        self.btn_graph.clicked.connect(self.open_graph)

        # Telemetry timer 20Hz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(50)

    def move(self, direction):
        global goal
        with state_lock:
            goal = max(MIN_REV, min(MAX_REV, goal + (STEP_REV if direction == "RIGHT" else -STEP_REV)))
            send_goal(goal)

    def zero(self):
        global zero, goal
        with state_lock:
            zero = read_raw(DXL_ID)
            goal = 0.0
            send_goal(goal)

    def open_graph(self):
        dialog = GraphDialog()
        dialog.exec()

    def update_telemetry(self):
        with state_lock:
            telemetry = read_telemetry(DXL_ID, zero)
        self.pos_label.setText(f"Position (rev): {telemetry[0]:.3f}")
        self.status.setText(f"Current Position: {telemetry[0]:.3f} rev | Velocity: {telemetry[1]:.2f} rpm")

# ─────────────────────────────────────────── MAIN
app = QApplication(sys.argv)
app.setFont(QFont("Consolas", 10))
window = MainWindow()
window.show()

try:
    sys.exit(app.exec())
finally:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()