#!/bin/bash
python3 make.py esp32 clean BOARD=ESP32_GENERIC_S3 DISPLAY=gc9a01 --enable-cdc-repl=y INDEV=cst816s IMU=qmi8658c
