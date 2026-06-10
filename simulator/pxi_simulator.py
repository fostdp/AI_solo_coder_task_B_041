#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PXI 采集卡模拟器 - 模拟 UDP 高速数据流
模拟 6 台混流式水轮机，每台 12 个水听器 + 8 个加速度计
每 1ms 上报一次数据包
"""

import socket
import struct
import time
import random
import math
import threading
import argparse
import signal
import sys
from dataclasses import dataclass
from enum import Enum
import numpy as np


class SensorType(Enum):
    HYDROPHONE = 0
    ACCELEROMETER = 1


class SensorPosition(Enum):
    SPIRAL_CASE_INLET = 0
    GUIDE_VANE = 1
    DRAFT_TUBE = 2
    RUNNER_BLADE = 3
    GUIDE_BEARING = 4
    THRUST_BEARING = 5


TURBINE_COUNT = 6
HYDROPHONE_COUNT = 12
ACCELEROMETER_COUNT = 8
BLADE_COUNT = 15

HYDROPHONE_SAMPLE_RATE = 51200
ACCELEROMETER_SAMPLE_RATE = 25600
SAMPLES_PER_PACKET = 128
PACKET_INTERVAL = 0.001

HYDROPHONE_POSITIONS = [
    SensorPosition.SPIRAL_CASE_INLET,
    SensorPosition.SPIRAL_CASE_INLET,
    SensorPosition.SPIRAL_CASE_INLET,
    SensorPosition.SPIRAL_CASE_INLET,
    SensorPosition.GUIDE_VANE,
    SensorPosition.GUIDE_VANE,
    SensorPosition.GUIDE_VANE,
    SensorPosition.GUIDE_VANE,
    SensorPosition.DRAFT_TUBE,
    SensorPosition.DRAFT_TUBE,
    SensorPosition.DRAFT_TUBE,
    SensorPosition.DRAFT_TUBE,
]

ACCELEROMETER_POSITIONS = [
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
    SensorPosition.RUNNER_BLADE,
]


@dataclass
class UDPDataPacket:
    timestamp: int
    batch_id: int
    turbine_id: int
    sensor_type: int
    sensor_id: int
    sensor_position: int
    blade_id: int
    sample_count: int
    sample_rate: int
    amplitude: float
    data: list

    def to_bytes(self):
        header = struct.pack(
            '<QIBBBBBHHf',
            self.timestamp,
            self.batch_id,
            self.turbine_id,
            self.sensor_type,
            self.sensor_id,
            self.sensor_position,
            self.blade_id,
            self.sample_count,
            self.sample_rate,
            self.amplitude
        )
        data_bytes = struct.pack('<' + 'f' * len(self.data), *self.data)
        return header + data_bytes


class SignalSimulator:
    def __init__(self):
        self.cavitation_levels = {}
        self.vibration_levels = {}
        for turbine_id in range(TURBINE_COUNT):
            self.cavitation_levels[turbine_id] = random.uniform(0.1, 0.4)
            self.vibration_levels[turbine_id] = random.uniform(0.05, 0.2)

    def update_levels(self):
        for turbine_id in range(TURBINE_COUNT):
            delta = random.uniform(-0.01, 0.01)
            self.cavitation_levels[turbine_id] = max(0.05, min(0.95, self.cavitation_levels[turbine_id] + delta))
            
            if self.cavitation_levels[turbine_id] > 0.8:
                delta_vib = random.uniform(0, 0.02)
                self.vibration_levels[turbine_id] = min(1.0, self.vibration_levels[turbine_id] + delta_vib)
            else:
                delta_vib = random.uniform(-0.005, 0.005)
                self.vibration_levels[turbine_id] = max(0.05, min(0.5, self.vibration_levels[turbine_id] + delta_vib))

    def generate_hydrophone_signal(self, turbine_id, sensor_id, sensor_position, n_samples, sample_rate):
        cavitation = self.cavitation_levels[turbine_id]
        
        signal = np.zeros(n_samples)
        t = np.arange(n_samples) / sample_rate
        
        base_freq = 100 + sensor_id * 50
        for harmonic in range(1, 6):
            amplitude = 1.0 / harmonic
            phase = random.uniform(0, 2 * math.pi)
            signal += amplitude * np.sin(2 * math.pi * base_freq * harmonic * t + phase)
        
        if cavitation > 0.3:
            cavitation_freq = 1000 + random.randint(0, 10) * 1000
            cavitation_amp = (cavitation - 0.3) * 5
            phase = random.uniform(0, 2 * math.pi)
            signal += cavitation_amp * np.sin(2 * math.pi * cavitation_freq * t + phase)
            
            noise_amp = cavitation * 3
            signal += noise_amp * np.random.randn(n_samples)
        
        signal += 0.5 * np.random.randn(n_samples)
        
        amplitude = float(np.max(np.abs(signal)))
        return signal.tolist(), amplitude

    def generate_accelerometer_signal(self, turbine_id, sensor_id, blade_id, n_samples, sample_rate):
        vibration = self.vibration_levels[turbine_id]
        cavitation = self.cavitation_levels[turbine_id]
        
        signal = np.zeros(n_samples)
        t = np.arange(n_samples) / sample_rate
        
        rot_freq = 2.0
        for harmonic in range(1, 10):
            amplitude = 1.0 / (harmonic ** 1.5)
            phase = random.uniform(0, 2 * math.pi)
            signal += amplitude * np.sin(2 * math.pi * rot_freq * harmonic * t + phase)
        
        blade_pass_freq = rot_freq * BLADE_COUNT
        blade_amp = vibration * 3
        phase = random.uniform(0, 2 * math.pi)
        signal += blade_amp * np.sin(2 * math.pi * blade_pass_freq * t + phase)
        
        if cavitation > 0.5:
            impact_freq = 5000 + random.randint(0, 5) * 1000
            impact_amp = (cavitation - 0.5) * 8
            num_impacts = int(cavitation * 10)
            for i in range(num_impacts):
                impact_start = random.randint(0, n_samples - 10)
                impact_len = min(20, n_samples - impact_start)
                impact_t = np.arange(impact_len) / sample_rate
                signal[impact_start:impact_start + impact_len] += impact_amp * np.exp(-impact_t * 1000) * np.sin(2 * math.pi * impact_freq * impact_t)
        
        signal += 0.3 * np.random.randn(n_samples)
        
        amplitude = float(np.max(np.abs(signal)))
        return signal.tolist(), amplitude


class PXISimulator:
    def __init__(self, host='127.0.0.1', port=9000, send_interval=PACKET_INTERVAL):
        self.host = host
        self.port = port
        self.send_interval = send_interval
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.signal_simulator = SignalSimulator()
        self.batch_id = 0
        self.stats = {
            'packets_sent': 0,
            'bytes_sent': 0,
            'start_time': 0
        }
        self.lock = threading.Lock()

    def send_packet(self, packet):
        data = packet.to_bytes()
        self.sock.sendto(data, (self.host, self.port))
        with self.lock:
            self.stats['packets_sent'] += 1
            self.stats['bytes_sent'] += len(data)

    def turbine_sender_thread(self, turbine_id):
        sensors = []
        
        for hydrophone_id in range(HYDROPHONE_COUNT):
            sensors.append({
                'type': SensorType.HYDROPHONE,
                'id': hydrophone_id,
                'position': HYDROPHONE_POSITIONS[hydrophone_id].value,
                'blade_id': 0,
                'sample_rate': HYDROPHONE_SAMPLE_RATE
            })
        
        for accelerometer_id in range(ACCELEROMETER_COUNT):
            blade_id = (accelerometer_id % BLADE_COUNT) + 1
            sensors.append({
                'type': SensorType.ACCELEROMETER,
                'id': accelerometer_id,
                'position': ACCELEROMETER_POSITIONS[accelerometer_id].value,
                'blade_id': blade_id,
                'sample_rate': ACCELEROMETER_SAMPLE_RATE
            })

        while self.running:
            cycle_start = time.time()
            timestamp = int(time.time() * 1000)
            
            with self.lock:
                self.batch_id += 1
                current_batch = self.batch_id
            
            for sensor in sensors:
                if not self.running:
                    break
                
                if sensor['type'] == SensorType.HYDROPHONE:
                    data, amplitude = self.signal_simulator.generate_hydrophone_signal(
                        turbine_id, sensor['id'], sensor['position'],
                        SAMPLES_PER_PACKET, sensor['sample_rate']
                    )
                else:
                    data, amplitude = self.signal_simulator.generate_accelerometer_signal(
                        turbine_id, sensor['id'], sensor['blade_id'],
                        SAMPLES_PER_PACKET, sensor['sample_rate']
                    )
                
                packet = UDPDataPacket(
                    timestamp=timestamp,
                    batch_id=current_batch,
                    turbine_id=turbine_id,
                    sensor_type=sensor['type'].value,
                    sensor_id=sensor['id'],
                    sensor_position=sensor['position'],
                    blade_id=sensor['blade_id'],
                    sample_count=SAMPLES_PER_PACKET,
                    sample_rate=sensor['sample_rate'],
                    amplitude=amplitude,
                    data=data
                )
                
                self.send_packet(packet)
            
            cycle_time = time.time() - cycle_start
            sleep_time = max(0, self.send_interval - cycle_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def level_updater_thread(self):
        while self.running:
            self.signal_simulator.update_levels()
            time.sleep(0.5)

    def stats_printer_thread(self):
        self.stats['start_time'] = time.time()
        while self.running:
            time.sleep(5)
            with self.lock:
                elapsed = time.time() - self.stats['start_time']
                pps = self.stats['packets_sent'] / elapsed if elapsed > 0 else 0
                bps = self.stats['bytes_sent'] / elapsed if elapsed > 0 else 0
                mbps = bps * 8 / 1e6
                
                cav_levels = {tid: f"{self.signal_simulator.cavitation_levels[tid]:.2f}" 
                             for tid in range(TURBINE_COUNT)}
                vib_levels = {tid: f"{self.signal_simulator.vibration_levels[tid]:.2f}" 
                             for tid in range(TURBINE_COUNT)}
            
            print(f"\r[PXI Sim] PPS: {pps:.0f} | Rate: {mbps:.2f} Mbps | "
                  f"Total: {self.stats['packets_sent']} packets ({self.stats['bytes_sent'] / 1e6:.1f} MB) | "
                  f"Cavitation: {cav_levels} | "
                  f"Vibration: {vib_levels}", end='', flush=True)

    def start(self):
        self.running = True
        print(f"PXI Simulator started, sending to {self.host}:{self.port}")
        print(f"Interval: {self.send_interval * 1000}ms | "
              f"{TURBINE_COUNT} turbines x {HYDROPHONE_COUNT + ACCELEROMETER_COUNT} sensors = "
              f"{TURBINE_COUNT * (HYDROPHONE_COUNT + ACCELEROMETER_COUNT)} packets/ms")
        print("Press Ctrl+C to stop")
        
        threads = []
        
        updater = threading.Thread(target=self.level_updater_thread, daemon=True)
        updater.start()
        threads.append(updater)
        
        stats_thread = threading.Thread(target=self.stats_printer_thread, daemon=True)
        stats_thread.start()
        threads.append(stats_thread)
        
        for turbine_id in range(TURBINE_COUNT):
            t = threading.Thread(target=self.turbine_sender_thread, args=(turbine_id,), daemon=True)
            t.start()
            threads.append(t)
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping simulator...")
            self.stop()

    def stop(self):
        self.running = False
        time.sleep(0.5)
        self.sock.close()
        
        with self.lock:
            elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 1
            print("\n========================================")
            print("PXI Simulator Final Stats:")
            print(f"  Packets sent:    {self.stats['packets_sent']}")
            print(f"  Bytes sent:      {self.stats['bytes_sent']} ({self.stats['bytes_sent'] / 1e9:.2f} GB)")
            print(f"  Duration:        {elapsed:.1f}s")
            print(f"  Average rate:    {self.stats['packets_sent'] / elapsed:.0f} packets/s")
            print(f"  Throughput:      {self.stats['bytes_sent'] * 8 / elapsed / 1e6:.2f} Mbps")
            print("========================================")


def signal_handler(signum, frame):
    print("\nReceived signal, shutting down...")
    if simulator:
        simulator.stop()
    sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PXI Data Acquisition Simulator')
    parser.add_argument('-H', '--host', default='127.0.0.1', help='Target host (default: 127.0.0.1)')
    parser.add_argument('-p', '--port', type=int, default=9000, help='Target port (default: 9000)')
    parser.add_argument('-i', '--interval', type=float, default=PACKET_INTERVAL,
                        help=f'Send interval in seconds (default: {PACKET_INTERVAL})')
    args = parser.parse_args()

    simulator = PXISimulator(args.host, args.port, args.interval)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    simulator.start()
