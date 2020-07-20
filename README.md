# Robotic-Vehicle-Off-Road
This project deals with building an FWD rover that can work in two different modes:  
- Autonomous: the rover has to move autonomously following a line drawn on
the roadway.  
- Manual: the rover has to be controlled by an external device using a serial
communication protocol.  

Furthermore, the developed firmware has to:  
- manage the wheels speed rotation requested by the user so that the rover can
move at a steady speed, regardless of the road gradients.  
- provide a serial communication protocol that allows to choose the driving
mode, to give driving commands and finally to manage sensors and actuators
linked to the rover.  

This project is designed for STMicroelectronics STM32-F401RE.  

All details in this [white paper](https://github.com/gdiprisco/Robotic-Vehicle-Off-Road/blob/master/RoVeR%20Robotic%20Vehicle%20off%20Road.pdf).
