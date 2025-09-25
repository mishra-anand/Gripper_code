# Gripper_code-
Integrated vision and tactile sensing and control
Purpose / brief description
Python code for a soft robotic gripper that fuses monocular vision and embedded tactile/pressure sensing to detect object shape/size and control pneumatic actuation (open/close/hold). Core scripts include camera acquisition, detection, size/shape estimation, and pressure/LED control combined in an “integrated_code.py” runner.

This repository contains Python scripts that combine **camera-based vision**, **LED/pressure tactile sensing**, and shape/size detection for object grasping and manipulation.
---
## System Requirements
Required Hardware: Raspberry Pi 3B or 4B, Raspberry Pi Camera Module, analog pressure sensor, ADS 1115 ADC converter
#Install Dependencies (additional beyond what the Raspberry Pi would already have on it)
# Install Python dependencies
pip3 install opencv-python numpy adafruit-circuitpython-ads1x15 adafruit-blinka matplotlib
# Install system dependencies (for Raspberry Pi camera support)

sudo apt-get install python-picamera python3-picamera

---
## :open_file_folder: Repository Contents
- `camera_tos.py` – Camera initialization and data acquisition.
- `general_detector.py` – Object detection utilities.
- `integrated_code.py` – Main script integrating vision and tactile sensing.
- `integrated_code_led.py` – Integrated code version using LED-based tactile sensing.
- `led_sensor_code.py` – LED tactile sensor control and reading.
- `pressure_control.py` – Control routines for pressure-based tactile sensing.
- `pressure_control_led.py` – LED-based pressure control.
- `shape_and_size_estimate.py` – Shape and size estimation algorithms.
- `shape_detection.py` – Shape detection functions.
---
## :gear: Installation
Clone the repository:
```bash
git clone https://github.com/mishra-anand/Gripper_code.git
cd Gripper_code
---
## Demo
To run this, you need to build the physical hardware (such as tactile sensors) and execute the code on the data using the instructions in the next section. As a quick demo check, you can connect a PiCamera to the Raspberry Pi and run the shape_and_size_estimate.py or shape_detection.py code to check if it is working.
---
## Usage
These scripts can be run separately for individual component testing or as part of a full system test inside the directory. There is no dataset to provide here because these scripts generate output based on real-time sensor (pressure, curvature, tactile) data. They do not use any offline data or datasets.
'python3 integrated_code.py'
'python3 pressure_control.py'
