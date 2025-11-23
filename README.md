# Embedded-ParkingSystem

A PIC18F8722-based embedded system that manages a multi-storey parking lot through serial communication with a Python simulator. 

## Overview
The microcontroller communicates with the simulator via USART (115200 bps, 8N1) and handles:
- Parking and exit commands  
- Subscription requests  
- Space assignment & fee calculation  
- 7-segment display modes (money / empty spaces)  
- ADC-based level selection  
- Timer and interrupt-driven periodic message sending (every 100ms)

The parking lot has 4 levels (A–D), each with 10 spaces, and supports subscriptions, a waiting queue, and real-time status reporting.

## Running the Simulator
```bash
python3 cengParkSimulator.py
```

## Build

Compile with MPLAB X + XC8 for PIC18F8722 @ 40MHz.

## Notes

Messages follow the $...# protocol and are sent every 100ms.
ADC determines active parking level.
PORTB interrupt toggles display mode.
