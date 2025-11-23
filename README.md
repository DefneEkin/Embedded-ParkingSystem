# Embedded-ParkingSystem
Embedded system for managing a multi-storey parking lot using PIC18F8722 with USART, ADC, timer interrupts, and real-time message handling.



A PIC18F8722-based embedded system that manages a multi-storey parking lot through serial communication with a Python simulator. Implemented for CENG336 Take-Home Exam 3.

## Overview
The microcontroller communicates with the simulator via USART (115200 bps, 8N1) and handles:
- Parking and exit commands  
- Subscription requests  
- Space assignment & fee calculation  
- 7-segment display modes (money / empty spaces)  
- ADC-based level selection  
- Timer and interrupt-driven periodic message sending (every 100ms)

The parking lot has 4 levels (A–D), each with 10 spaces, and supports subscriptions, a waiting queue, and real-time status reporting.

## Directory Structure

