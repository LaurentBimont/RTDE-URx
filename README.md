# RTDE-URx
RTDE class to control a Universal Robot

This class is built on the RTDE package proposed by Universal Robots at https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/

A python script (RTDEclass.py) and a ur-script (RTDEcontrolloop) are running on a computer/robot. They communicate throught registers defined in control_loop_configuration.xml.

Registers can be written/read by one script and read/written by the other.
* State (from UR to python)
  * Actual_TCP_pose : Actual TCP pose
  * Output_int_register_0 : set to 0 when a loop in UR script is finished, 1 otherwise
  
* Setp (from python to UR)
  * X, Y, Z, Rx, Ry, Rz : Target cartesian coordinates
  
* Gripper (from python to UR)
  * input_int_register_1: 1 : close, 2: open
  
* Watchdog (from python to UR)
  * input_int_register_0 : To keep link active between python and UR
