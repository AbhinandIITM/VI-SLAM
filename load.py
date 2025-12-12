#!/usr/bin/env python3
# euroc_dataset_maker.py - Records synchronized IMU + Camera with STRICT IMU data validation

import asyncio
import websockets
import cv2
import numpy as np
import time
import os
import threading
import shutil
from datetime import datetime
from collections import deque
import json

# Euroc MH dataset format
DATASET_DIR = "euroc_dataset"
CAMERA_ID = 2
IMU_WS_URL = "ws://localhost:8001"
RECORD_DURATION = 60
IMU_CHECK_DURATION = 5.0
MIN_IMU_SAMPLES_CHECK = 20  # Require 20+ valid samples before recording

# Euroc camera calibration
CAM0_FX = 458.654
CAM0_FY = 457.296
CAM0_CX = 367.215
CAM0_CY = 248.375
CAM0_WIDTH = 640
CAM0_HEIGHT = 480

class EurocDatasetMaker:
    def __init__(self):
        self.dataset_dir = DATASET_DIR
        self.cam_dir = None  # Don't create until IMU confirmed
        self.imu_data = deque(maxlen=20000)
        self.cam_timestamps = []
        self.imu_timestamps = []
        self.start_time = None
        self.recording = False
        self.imu_samples_received = 0
        self.imu_connected = False
        self.valid_imu_samples = 0
        
        # Camera setup (but don't create dirs yet)
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM0_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM0_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 20.0)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"❌ Camera {CAMERA_ID} not available!")
        print(f"✅ Camera {CAMERA_ID} ready ({CAM0_WIDTH}x{CAM0_HEIGHT}@20Hz)")
    
    def validate_imu_sample(self, ax, ay, az, wx, wy, wz):
        """Validate IMU sample physics"""
        # Gravity check: Z accel should be around ±9.8g when stationary
        gravity_ok = abs(abs(az) - 9.81) < 3.0 or abs(abs(ay) - 9.81) < 3.0
        
        # Gyro reasonable bounds
        gyro_ok = all(abs(w) < 10.0 for w in [wx, wy, wz])  # deg/s
        
        # Accel reasonable bounds  
        accel_ok = all(abs(a) < 50.0 for a in [ax, ay, az])
        
        return gravity_ok and gyro_ok and accel_ok
    
    async def check_imu_connection(self):
        """STRICT IMU validation - NO DATA SAVED UNTIL THIS PASSES"""
        print(f"\n🔍 STRICT IMU VALIDATION at {IMU_WS_URL} for {IMU_CHECK_DURATION}s...")
        print("📡 echo_imu_server.py MUST be running with VALID DATA")
        print("⏳ Hold sensor still for gravity check...")
        
        start_check = time.time()
        consecutive_failures = 0
        max_failures = 5
        
        try:
            async with asyncio.timeout(IMU_CHECK_DURATION + 2):
                async with websockets.connect(IMU_WS_URL, timeout=3) as websocket:
                    print("✅ IMU websocket connected")
                    self.imu_connected = True
                    
                    async for message in websocket:
                        try:
                            # Parse IMU
                            data = message.strip().split(',')
                            if len(data) == 7:
                                ts, ax, ay, az, wx, wy, wz = map(float, data)
                            else:
                                data_json = json.loads(message)
                                ts = data_json.get('timestamp', time.time())
                                accel = data_json.get('accel', [0,0,0])
                                gyro = data_json.get('gyro', [0,0,0])
                                ax, ay, az = accel
                                wx, wy, wz = gyro
                            
                            self.imu_samples_received += 1
                            
                            # Validate physics
                            if self.validate_imu_sample(ax, ay, az, wx, wy, wz):
                                self.valid_imu_samples += 1
                                consecutive_failures = 0
                                print(f"   ✅ #{self.valid_imu_samples}: "
                                      f"ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}g "
                                      f"(gyro: {wx:.1f},{wy:.1f},{wz:.1f})")
                            else:
                                consecutive_failures += 1
                                print(f"   ⚠️  #{self.imu_samples_received}: "
                                      f"ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}g "
                                      f"[INVALID PHYSICS - {consecutive_failures}/{max_failures}]")
                            
                            # Early success
                            if self.valid_imu_samples >= MIN_IMU_SAMPLES_CHECK:
                                print(f"\n🎉 EARLY SUCCESS: {self.valid_imu_samples} valid samples!")
                                break
                                
                        except Exception as e:
                            print(f"   ❌ Parse error: {e}")
                            consecutive_failures += 1
                        
                        # Check for too many bad samples
                        if consecutive_failures >= max_failures:
                            print(f"\n❌ TOO MANY INVALID SAMPLES ({consecutive_failures}/{max_failures})")
                            return False
                            
        except asyncio.TimeoutError:
            print("\n⏰ IMU check timeout - no/weak data flow")
        except Exception as e:
            print(f"\n❌ IMU connection failed: {e}")
            self.imu_connected = False
        
        duration = time.time() - start_check
        rate = self.imu_samples_received / duration if duration > 0 else 0
        valid_rate = self.valid_imu_samples / duration if duration > 0 else 0
        
        print(f"\n📊 FINAL IMU VALIDATION ({duration:.1f}s):")
        print(f"   Total samples: {self.imu_samples_received}")
        print(f"   Valid samples: {self.valid_imu_samples}")
        print(f"   Rate: {rate:.1f} Hz (valid: {valid_rate:.1f} Hz)")
        
        success = (self.imu_connected and 
                  self.valid_imu_samples >= MIN_IMU_SAMPLES_CHECK and 
                  valid_rate > 5.0)  # >5Hz valid data
        
        if success:
            print("✅✅ IMU VALIDATED! Creating dataset directories...")
            self.cam_dir = os.path.join(self.dataset_dir, "cam0", "data")
            os.makedirs(self.cam_dir, exist_ok=True)
            os.makedirs(os.path.join(self.dataset_dir, "cam0"), exist_ok=True)
            return True
        else:
            print("❌❌ IMU VALIDATION FAILED - NO DATASET CREATED")
            print("💡 Check: echo_imu_server.py running? Sensor connected? Units correct?")
            return False
    
    def imu_callback(self, message):
        """Store validated IMU data during recording"""
        try:
            data = message.strip().split(',')
            if len(data) == 7:
                ts, ax, ay, az, wx, wy, wz = map(float, data)
            else:
                data_json = json.loads(message)
                accel = data_json.get('accel', [0,0,0])
                gyro = data_json.get('gyro', [0,0,0])
                ax, ay, az = accel
                wx, wy, wz = gyro
            
            # Only store VALID physics data
            if self.validate_imu_sample(ax, ay, az, wx, wy, wz):
                current_time = time.time()
                self.imu_data.append((current_time, ax, ay, az, wx, wy, wz))
                self.imu_timestamps.append(current_time)
        except:
            pass  # Silently drop bad data
    
    async def imu_websocket(self):
        """Background IMU collection"""
        while self.recording:
            try:
                async with websockets.connect(IMU_WS_URL) as websocket:
                    async for message in websocket:
                        if self.recording:
                            self.imu_callback(message)
            except:
                await asyncio.sleep(0.5)
    
    def camera_thread(self):
        """Camera recording"""
        frame_idx = 0
        while self.recording:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
                
            current_time = time.time()
            timestamp_file = os.path.join(self.cam_dir, f"{int(current_time*1e9):020d}.png")
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(timestamp_file, gray)
            self.cam_timestamps.append(current_time)
            
            if frame_idx % 20 == 0:
                print(f"📸 {len(self.cam_timestamps)} frames | "
                      f"IMU buffer: {len(self.imu_data)}")
            frame_idx += 1
            
            time.sleep(0.05)
    
    def save_dataset(self):
        """Save only if we have good data"""
        if len(self.imu_data) < 1000:
            print("❌ ABORT: Insufficient IMU data!")
            return
        
        print("💾 Saving validated Euroc dataset...")
        
        # Timestamps
        with open(os.path.join(self.dataset_dir, "cam0", "times.txt"), 'w') as f:
            for ts in self.cam_timestamps:
                f.write(f"{ts:.9f}\n")
        
        with open(os.path.join(self.dataset_dir, "times.txt"), 'w') as f:
            for ts in self.imu_timestamps:
                f.write(f"{ts:.9f}\n")
        
        # IMU (only validated data)
        with open(os.path.join(self.dataset_dir, "imu0.txt"), 'w') as f:
            f.write("# timestamp accel_x accel_y accel_z gyro_x gyro_y gyro_z\n")
            for ts, ax, ay, az, wx, wy, wz in self.imu_data:
                f.write(f"{ts:.9f} {ax:.6f} {ay:.6f} {az:.6f} {wx:.6f} {wy:.6f} {wz:.6f}\n")
        
        # state.yaml
        state_yaml = f"""%YAML:1.0
---
imudata: "imu0.txt"
cam0data: "cam0/data/"
cam0:
  cam0_imu_timestamp_sync: 0.0
  T_cam0_imu: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1.0, 0.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0,  0.0, 0.0, 1.0, 0.0,  0.0, 0.0, 0.0, 1.0]
  dist_coef: [0.0, 0.0, 0.0, 0.0, 0.0]
  resolution: [{CAM0_WIDTH}, {CAM0_HEIGHT}]
  fx: {CAM0_FX}
  fy: {CAM0_FY}
  cx: {CAM0_CX}
  cy: {CAM0_CY}
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
grav: 9.81
"""
        with open(os.path.join(self.dataset_dir, "state.yaml"), 'w') as f:
            f.write(state_yaml)
        
        print(f"✅✅ Dataset COMPLETE: {len(self.cam_timestamps)} images, {len(self.imu_data)} IMU")
    
    async def start_recording(self, duration=RECORD_DURATION):
        """Main flow: IMU check → THEN record → THEN save"""
        print("🚀 Euroc Dataset Maker - STRICT IMU VALIDATION")
        
        # CRITICAL: Check IMU FIRST, no directories created yet
        if not await self.check_imu_connection():
            response = input("\n❌ IMU FAILED VALIDATION. Force continue? (y/N): ")
            if response.lower() != 'y':
                print("👋 Aborted - no dataset created")
                return
            print("⚠️  FORCED CONTINUE - may produce bad data!")
            self.cam_dir = os.path.join(self.dataset_dir, "cam0", "data")
            os.makedirs(self.cam_dir, exist_ok=True)
            os.makedirs(os.path.join(self.dataset_dir, "cam0"), exist_ok=True)
        
        print(f"\n🎬 RECORDING {duration}s - MOVE SENSOR NOW!")
        self.start_time = time.time()
        self.recording = True
        
        # Start parallel recording
        cam_thread = threading.Thread(target=self.camera_thread, daemon=True)
        imu_task = asyncio.create_task(self.imu_websocket())
        cam_thread.start()
        
        await asyncio.sleep(duration)
        
        self.recording = False
        cam_thread.join(timeout=2)
        imu_task.cancel()
        
        self.save_dataset()
        self.cap.release()
        print("🎉 Done! Ready for ORB_SLAM3 (update T_cam0_imu)")

async def main():
    if os.path.exists(DATASET_DIR):
        shutil.rmtree(DATASET_DIR)
        print("🗑️  Cleaned previous dataset")
    
    maker = EurocDatasetMaker()
    await maker.start_recording(RECORD_DURATION)

if __name__ == "__main__":
    asyncio.run(main())
