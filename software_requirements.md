# Software Requirements

## 1. General:
- Windows 10 or higher https://www.microsoft.com/de-de/software-download/windows8ISO (Linux maybe possible, but not tested)
- Python 3.9 or higher https://www.python.org/downloads/release/python-390/ (for gripper control)
- pySerial 3.4 or higher https://pyserial.readthedocs.io/en/latest/pyserial.html (for gripper control)
- IntelReal Sense Driver https://www.intel.de/content/www/de/de/download/19236/intel-realsense-d400-series-universal-windows-platform-uwp-driver-for-windows-10.html
- IntelReal Sense SDK 2.0 https://www.intelrealsense.com/sdk-2/ (for image processing and calibration)
- RobotStudio 2022.1 https://new.abb.com/products/robotics/de/robotstudio/downloads (to programm robot controller)

## 2. Matlab:
- Matlab R2021a or higher with activated python environment, see https://de.mathworks.com/help/matlab/matlab_external/create-object-from-python-class.html for reference
- Instrument Control Toolbox https://de.mathworks.com/products/instrument.html
- Robotiq Gripper Toolbox https://github.com/MinyaRancic/Robotiq-Gripper (for gripper control)
- Image Processing Toolbox https://de.mathworks.com/products/image.html
- Computer Vision Toolbox https://www.mathworks.com/products/computer-vision.html
- Aerospace Toolbox https://www.mathworks.com/products/aerospace-toolbox.html
- Data Acquisition Toolbox https://www.mathworks.com/products/data-acquisition.html

## 3. Provided
- gripper_init.m (for gripper intialisation and calibration)
- image_processing.m (for image processing, communication with controller and control of the gripper)
- calib_data.txt (Workobject and Tool definition used in the robot controller)
- main.txt (Main programm routine used in the robot controller)
- mod_1.txt (Position data used in the robot controller)
