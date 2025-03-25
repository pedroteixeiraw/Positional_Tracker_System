# Positional Tracking System with Microcontroller  

This project focuses on developing a system that measures and graphically represents human displacement in real time. It utilizes the Microchip PIC18F47Q10 microcontroller, the MMA7361L accelerometer, and MATLAB for data processing and visualization.  

## Signal Acquisition  

The acquisition system consists of a board equipped with an MMA7361L accelerometer, which provides three output signals. Each output corresponds to a voltage signal that is later converted into an instantaneous acceleration value along the x-axis (Output 1), y-axis (Output 2), and z-axis (Output 3).  

These three output signals from the accelerometer board are connected to three digital inputs (RD7, RD6, and RD5) on the Microchip "Curiosity HPC" board, which integrates the PIC18F47Q10 microcontroller. The system samples acceleration values at a frequency of 93 Hz for each input pin.  

Once sampled, the values are converted from decimal to binary format and then transmitted to a computer via the EUSART1 module on the Curiosity HPC board. This data is subsequently processed and visualized using MATLAB.  


**Tools Stack:** Microchip MPLAB, MatLab, C, Assembly