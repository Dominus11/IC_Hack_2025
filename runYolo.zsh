#!/bin/zsh

cd "/Users/sarath/Downloads/Projects/IC_Hack_2025/yolov9"
python detect.py --weights gelan-c.pt --conf 0.5 --source 2 --device mps