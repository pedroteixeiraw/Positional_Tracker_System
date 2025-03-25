# Positional Tracking System with Microcontroller  

This project focuses on developing a system that measures and graphically represents human displacement in real time. It is based on the Microchip PIC18F47Q10 microcontroller, the MMA7361L accelerometer, and MATLAB for data processing and visualization.  

## Signal Acquisition  

The acquisition system consists of a Microchip PIC18F47Q10 microcontroller connected to an MMA7361L accelerometer, which generates three output signals. Each signal corresponds to a voltage value that is later converted into an instantaneous acceleration measurement along the x-axis, y-axis, and z-axis.  
These three output signals from the accelerometer board are connected to digital inputs (RD7, RD6, and RD5) on the Microchip "Curiosity HPC" board, which integrates the PIC18F47Q10 microcontroller. The system samples acceleration values at a frequency of 93 Hz for each input pin.  

Once sampled, the values are converted from decimal to binary format and transmitted to a computer via the EUSART1 module on the Curiosity HPC board. MATLAB is then used for data processing and visualization.  

**Tools Stack:** Microchip MPLAB, MatLab, C, Assembly
