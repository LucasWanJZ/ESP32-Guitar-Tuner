Audio Processing with I2S and FFT on ESP32

This repository contains an implementation of an audio processing system using I2S and FFT on an ESP32 microcontroller. The project reads audio data through an I2S microphone, performs Fast Fourier Transform (FFT) to detect the dominant frequency, and identifies musical notes based on the detected frequencies.
Table of Contents

    Hardware Requirements
    Software Requirements
    Installation
    Usage
    Code Overview
    Tasks Description
    Functions Description
    Notes Detection
    License

Hardware Requirements

    ESP32 microcontroller
    I2S microphone (e.g., INMP441)
    Jumper wires
    Breadboard (optional)

Software Requirements

    Arduino IDE
    ESP32 board support in Arduino IDE
    Required libraries:
        driver/i2s.h
        arduinoFFT.h
        math.h

Installation

    Set up Arduino IDE for ESP32:
        Follow the instructions from the ESP32 Arduino Core to add ESP32 board support to the Arduino IDE.

    Install Libraries:
        Ensure the arduinoFFT library is installed. You can install it through the Arduino Library Manager.

    Clone the Repository:
        Clone this repository to your local machine.

    Open the Project:
        Open the .ino file in the Arduino IDE.

    Upload the Code:
        Connect your ESP32 to your computer and upload the code using the Arduino IDE.

Usage

    Connect the Hardware:
        Connect the I2S microphone to the ESP32 as follows:
            I2S_WS (Word Select) to GPIO 16
            I2S_DIN (Data In) to GPIO 14
            I2S_BCK (Bit Clock) to GPIO 17

    Upload and Run:
        Upload the code to the ESP32.
        Open the Serial Monitor in the Arduino IDE to view the output.

Code Overview

The main functionalities of this project are divided into tasks and helper functions:
Tasks Description

    ReadingTask:
        Continuously reads audio data from the I2S microphone.
        Stores the audio samples in a buffer for processing.

    ProcessingTask:
        Processes the audio data stored in the buffer.
        Performs FFT to detect the dominant frequency.
        Identifies musical notes based on the detected frequencies.

Functions Description

    setup():
        Initializes the Serial communication.
        Initializes the I2S interface.
        Calculates standard frequencies for musical notes.
        Creates tasks for reading and processing audio data.

    initI2S():
        Configures and starts the I2S interface.

    calcSF1():
        Calculates standard frequencies of musical notes in the 1st octave based on the A4 note (440 Hz).

    ReadingTask():
        Reads audio samples from the I2S microphone.
        Stores samples in a buffer for further processing.

    ProcessingTask():
        Performs FFT on the audio samples.
        Identifies the dominant frequency.
        Calls noteDetection() to identify the musical note.

    noteDetection():
        Determines the correct note and its octave from the detected frequency.
        Calculates deviation from the standard frequency and suggests tuning adjustments.

    calcSemitoneFromA4():
        Calculates the number of semitones from the detected frequency to A4 (440 Hz).

Notes Detection

    The detected frequency is compared against standard frequencies of musical notes.
    The program identifies the note, its octave, and suggests tuning adjustments if needed.
