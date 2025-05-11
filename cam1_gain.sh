#!/bin/bash
#./gain_exp_pid.py --gain-min 0 --gain-max 255 --device /dev/v4l/by-path/platform-fc800000.usb-usb-0\:1.2\:1.0-video-index0 --pipeline "rtspsrc location=rtsp://127.0.0.1:8554/cam1_lq latency=0 ! rtph265depay ! h265parse ! queue ! mppvideodec format=16 width=640 height=360 ! queue ! videoconvert ! videorate ! video/x-raw,framerate=5/1 !  appsink max-buffers=1 drop=true" --target 45
./controller --gain-min 0 --gain-max 255 --device /dev/v4l/by-path/platform-fc800000.usb-usb-0\:1.2\:1.0-video-index0 --pipeline "rtspsrc location=rtsp://127.0.0.1:8554/cam1_lq latency=0 ! rtph265depay ! h265parse ! queue ! mppvideodec format=16 width=640 height=360 ! queue ! videoconvert ! videorate ! video/x-raw,framerate=5/1 !  appsink max-buffers=1 drop=true" --target 45

