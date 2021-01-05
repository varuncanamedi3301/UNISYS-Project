# UNISYS-Project
Edge driven, IoT based, intelligent system for restricted access control for commercial establishments. This project is a part of UNISYS student innovation program. Developed by students of BMSCE, Bangaluru.


## Libraries and languages used :-
1. Python 3.6
2. Paho MQTT library
3. OpenCV-2
4. RPI
5. Picamera library for Raspberry Pi camera module
6. NumPy

## Hardware used :-
1. Raspberry Pi 4
2. LM-358 IR proximity sensor
3. Pi camera module 
4. USB power supply for Raspberry Pi
5. Breadboard (for prototyping purposes)

## Basic Overview :-

In this system, we will be using a standard MQTT protocol over the local WiFi network to establish communication between devices. The goal of the system would be to provide accurate identification and access, with minimal usage of confidential biometric data. To achieve this we will be using HOG(Histogram of Oriented Graphs) features generated at the sensor level as input to the neural network image recognition algorithm. The descision given by the algorithm will be transmitted to the all the gate control systems. 
