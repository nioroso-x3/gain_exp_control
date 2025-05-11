#!/usr/bin/env python3
import cv2
import numpy as np
import subprocess
import argparse
import time

def set_gain_v4l2(device_path, gain):
    """Set camera gain via v4l2-ctl."""
    subprocess.run(['v4l2-ctl', '-d', device_path, '--set-ctrl', f'gain={int(gain)}'],
                   check=False)

#Sets a fixed set of valid exposure values
def set_exp_abs_v4l2(device_path, exp):
    exp_map = (1, 2, 5, 10, 20, 39, 78)
    exp = exp_map[int(exp)]
    """Set camera gain via v4l2-ctl."""
    subprocess.run(['v4l2-ctl', '-d', device_path, '--set-ctrl', f'exposure_absolute={exp}'],
                   check=False)

def get_histogram_brightness(gray_frame):
    """Estimate brightness using median from grayscale histogram."""
    hist = cv2.calcHist([gray_frame], [0], None, [256], [0, 256])
    cumsum = np.cumsum(hist)
    total = gray_frame.size
    median = np.searchsorted(cumsum, total // 2)
    return median

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def main():
    parser = argparse.ArgumentParser(description="UVC Gain Control with PID Controller")
    parser.add_argument("--gain-min", type=int, required=True, help="Minimum gain value")
    parser.add_argument("--gain-max", type=int, required=True, help="Maximum gain value")
    parser.add_argument("--pipeline", type=str, required=True, help="GStreamer pipeline for OpenCV")
    parser.add_argument("--device", type=str, required=True, help="V4L2 device path (e.g. /dev/video0)")
    parser.add_argument("--target", type=int, default=100, help="Target brightness (0-255)")
    parser.add_argument("--kp", type=float, default=0.1, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=0.001, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.1, help="Derivative gain")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open video stream")
        return
    exposure = 3
    gain = (args.gain_max - args.gain_min) // 2
    previous_error = 0
    integral = 0
    last_time = time.time()
    frames = 0
    print("Starting PID gain control loop...")
    bv = []
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bv.append( get_histogram_brightness(gray))
        if (len(bv) < 2):
            continue
        if (len(bv) == 3):
            bv.pop(0)
        brightness = float(sum(bv)) / len(bv)
        now = time.time()
        dt = now - last_time 

        # PID controller
        error = args.target - brightness
        integral += error * dt
        derivative = (error - previous_error) / dt if dt > 0 else 0
        output = args.kp * error + args.ki * integral + args.kd * derivative

        # Apply output to gain directly
        gain += output
        gain = clamp(gain, args.gain_min, args.gain_max)
        
        # Only adjust exposure if gain saturates, but scale down the effect
        if gain == args.gain_max or gain == args.gain_min:
            if output != 0:
                exposure += output * 6 / (args.gain_max - args.gain_min)
        exposure = clamp(exposure, 0, 6)

        set_gain_v4l2(args.device, gain)
        set_exp_abs_v4l2(args.device, exposure)
        #if(frames % 5 == 0):
        #  print(f"O: {output} E: {int(error)} I: {integral} D: {derivative} gain: {gain} exp: {exposure}")
        previous_error = error
        last_time = now
        frames += 1
    cap.release()

if __name__ == "__main__":
    main()
