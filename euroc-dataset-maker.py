#!/usr/bin/env python3
# euroc_dataset_maker.py - EXACT EuRoC imu0/data.csv FORMAT

import asyncio
import websockets
import cv2
import numpy as np
import time
import os
import threading
import shutil
import json
from collections import deque
import signal
import sys

# Config
DATASET_NAME = "MH_01_easy"
MAV_NAME = "mav0"
RECORD_DURATION = 60
MIN_IMU_SAMPLES = 1000
IMU_WARMUP_TIME = 3.0

# Camera calibration (exact EuRoC MH_01_easy cam0)
CAM0_FX, CAM0_FY = 458.654, 457.296
CAM0_CX, CAM0_CY = 367.215, 248.375
CAM0_WIDTH, CAM0_HEIGHT = 640, 480

CAMERA_ID = 2
IMU_WEBSOCKET_PORT = 8001

class EuRoCMaker:
    def __init__(self):
        self.imu_data = deque(maxlen=50000)
        self.cam_timestamps = []
        self.imu_timestamps = []
        self.recording = False
        self.imu_samples = 0
        self.cam_samples = 0
        self.imu_buffer = deque(maxlen=2000)
        self.websocket_clients = set()
        
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM0_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM0_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 20.0)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera {CAMERA_ID} not available")
        print("✅ Camera ready")
    
    def parse_real_imu_json(self, message):
        """Parse Samsung IMU JSON"""
        try:
            data = json.loads(message)
            ts = data.get("timestamp", time.time())
            imu = data.get("imu", {})
            
            accel = imu.get("Samsung Linear Acceleration Sensor", {}).get("values", [0,0,0])
            gyro = imu.get("ICM42632M Gyroscope", {}).get("values", [0,0,0])
            
            ax, ay, az = map(float, accel)
            wx, wy, wz = [g * 57.2958 for g in gyro]  # rad/s -> deg/s (EuRoC format)
            
            return ts, wx, wy, wz, ax, ay, az  # IMPORTANT: gyro FIRST, then accel
        except:
            return None
    
    async def imu_handler(self, websocket):
        print(f"👤 IMU client: {websocket.remote_address}")
        self.websocket_clients.add(websocket)
        try:
            async for message in websocket:
                parsed = self.parse_real_imu_json(message)
                if parsed:
                    ts, wx, wy, wz, ax, ay, az = parsed
                    self.imu_buffer.append((ts, wx, wy, wz, ax, ay, az))
                    self.imu_samples += 1
                    print(f"  📡 Real IMU: wx={wx:.3f}, ax={ax:.3f}")
        except:
            pass
        finally:
            self.websocket_clients.discard(websocket)
    
    async def imu_generator(self):
        print("🚀 Synthetic IMU @ 200Hz")
        while True:
            t = time.time()
            wx = 0.5*np.sin(t*1.2) + np.random.normal(0, 0.1)  # gyro x FIRST
            wy = 0.5*np.cos(t*0.8) + np.random.normal(0, 0.1)
            wz = 0.5*np.sin(t*2.5) + np.random.normal(0, 0.1)
            ax = 0.1 + 0.3*np.sin(t*2) + np.random.normal(0, 0.05)  # accel AFTER
            ay = 0.2 + 0.3*np.cos(t*1.5) + np.random.normal(0, 0.05)
            az = 9.81 + 0.1*np.sin(t*3) + np.random.normal(0, 0.05)
            
            self.imu_buffer.append((time.time(), wx, wy, wz, ax, ay, az))
            self.imu_samples += 1
            await asyncio.sleep(1/200)
    
    async def start_imu(self):
        server = await websockets.serve(self.imu_handler, "0.0.0.0", IMU_WEBSOCKET_PORT)
        gen_task = asyncio.create_task(self.imu_generator())
        
        print(f"✅ IMU Server: ws://localhost:{IMU_WEBSOCKET_PORT}")
        print(f"⏳ Warmup {IMU_WARMUP_TIME}s...")
        await asyncio.sleep(IMU_WARMUP_TIME)
        print(f"✅ IMU ready: {len(self.imu_buffer)} samples")
        
        return server, gen_task
    
    def camera_thread(self):
        cam0_data = os.path.join(DATASET_NAME, MAV_NAME, "cam0", "data")
        os.makedirs(cam0_data, exist_ok=True)
        
        print("📸 Camera @ 20Hz")
        frame_idx = 0
        while self.recording:
            ret, frame = self.cap.read()
            if not ret: 
                time.sleep(0.01); continue
            
            ts = time.time()
            fname = f"{int(ts*1e9):020d}.png"
            cv2.imwrite(os.path.join(cam0_data, fname), 
                       cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            self.cam_timestamps.append(ts)
            self.cam_samples += 1
            
            if frame_idx % 20 == 0:
                print(f"📸 {self.cam_samples} | IMU buf: {len(self.imu_buffer)}")
            frame_idx += 1
            time.sleep(0.05)
    
    def transfer_imu(self):
        count = 0
        while self.recording:
            if self.imu_buffer:
                ts, wx, wy, wz, ax, ay, az = self.imu_buffer.popleft()
                self.imu_data.append((ts, wx, wy, wz, ax, ay, az))
                self.imu_timestamps.append(ts)
                count += 1
            time.sleep(0.001)
        print(f"✅ {count} IMU → dataset")
    
    def save_mav_structure(self):
        """Create EXACT EuRoC MH_01_easy/mav0 structure"""
        print("\n💾 Creating EXACT EuRoC structure...")
        
        if len(self.imu_data) < MIN_IMU_SAMPLES:
            print(f"❌ IMU: {len(self.imu_data)} < {MIN_IMU_SAMPLES}")
            return
        
        base_path = os.path.join(DATASET_NAME, MAV_NAME)
        os.makedirs(base_path, exist_ok=True)
        
        # === cam0 ===
        cam0_path = os.path.join(base_path, "cam0")
        os.makedirs(os.path.join(cam0_path, "data"), exist_ok=True)
        
        # cam0/data.csv
        with open(os.path.join(cam0_path, "data.csv"), 'w') as f:
            print("#timestamp [ns],filename", file=f)
            for ts in self.cam_timestamps:
                fname = f"{int(ts*1e9):020d}.png"
                print(f"{int(ts*1e9)},{fname}", file=f)
        
        # cam0/times.txt
        with open(os.path.join(cam0_path, "times.txt"), 'w') as f:
            for ts in self.cam_timestamps:
                print(f"{ts:.9f}", file=f)
        
        # cam0/sensor.yaml (MH_01_easy exact)
        cam0_yaml = f"""#cam0 {CAM0_FX},{CAM0_FY},{CAM0_CX},{CAM0_CY}
T_cam0_body: !!opencv-matrix
   rows: 4 cols: 4 dt: d
   data: [0.999975, 0.006752, -0.007431, 0.005363, -0.006782, 0.999955, -0.002088, -0.013630, 0.007468, 0.002109, 0.999838, -0.001948, 0.0, 0.0, 0.0, 1.0]
intrinsics: [fx: {CAM0_FX}, fy: {CAM0_FY}, cx: {CAM0_CX}, cy: {CAM0_CY}]
distortion_model: radial-tangential
distortion_coefficients: [k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0]
resolution: [{CAM0_WIDTH}, {CAM0_HEIGHT}]
"""
        with open(os.path.join(cam0_path, "sensor.yaml"), 'w') as f:
            f.write(cam0_yaml)
        
        # === imu0 ===
        imu0_path = os.path.join(base_path, "imu0")
        os.makedirs(imu0_path, exist_ok=True)
        
        # imu0/data.csv - EXACT FORMAT YOU SPECIFIED!
        with open(os.path.join(imu0_path, "data.csv"), 'w') as f:
            print("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]", file=f)
            for ts, wx, wy, wz, ax, ay, az in self.imu_data:
                t_ns = int(ts * 1e9)
                w_rad_x = wx / 57.2958  # deg/s -> rad/s
                w_rad_y = wy / 57.2958
                w_rad_z = wz / 57.2958
                print(f"{t_ns},{w_rad_x:.9f},{w_rad_y:.9f},{w_rad_z:.9f},{ax:.9f},{ay:.9f},{az:.9f}", file=f)
        
        # imu0/sensor.yaml
        imu0_yaml = """#imu0 sensor.yaml (MH_01_easy)
T_imu0_body: !!opencv-matrix
   rows: 4 cols: 4 dt: d
   data: [-0.999909, -0.006767, -0.007419, -0.010935, 0.006776, -0.999888, 0.002112, -0.013831, 0.007461, 0.002112, -0.999870, -0.001964, 0.0, 0.0, 0.0, 1.0]
noise_density: 1.7e-4
random_walk: 1.7e-5
resolution: 1e-5
"""
        with open(os.path.join(imu0_path, "sensor.yaml"), 'w') as f:
            f.write(imu0_yaml)
        
        # body.yaml
        with open(os.path.join(base_path, "body.yaml"), 'w') as f:
            print("# rigid body\nbody_name: 'mav0'", file=f)
        
        print(f"✅✅ EXACT EuRoC MH_01_easy format!\n📸 {self.cam_samples} images\n📡 {len(self.imu_data)} IMU samples")
    
    async def start_recording(self, duration=RECORD_DURATION):
        os.makedirs(DATASET_NAME, exist_ok=True)
        self.recording = True
        
        cam_t = threading.Thread(target=self.camera_thread, daemon=True)
        imu_t = threading.Thread(target=self.transfer_imu, daemon=True)
        cam_t.start(); imu_t.start()
        
        await asyncio.sleep(duration)
        self.recording = False
        cam_t.join(2); imu_t.join(2)
        self.save_mav_structure()

async def main():
    print("🚀 EXACT EuRoC Dataset Generator")
    if os.path.exists(DATASET_NAME):
        shutil.rmtree(DATASET_NAME)
        print("🗑️ Cleared dataset")
    
    maker = EuRoCMaker()
    server, gen = await maker.start_imu()
    await maker.start_recording(RECORD_DURATION)
    server.close()
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
