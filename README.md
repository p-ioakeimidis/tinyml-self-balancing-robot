# TinyML-Based Self-Balancing Robot Control with Arduino Nano 33 BLE Sense

## Overview

This project demonstrates the implementation of a **Tiny Machine Learning (TinyML)** model for controlling a two-wheeled self-balancing robot using the **Arduino Nano 33 BLE Sense**. The objective is to replace a traditional control algorithm with a lightweight neural network capable of performing real-time inference on resource-constrained embedded hardware.

<img width="454" height="576" alt="ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/2b2fea44-4ef4-424e-a115-6406bc5aebd4" />

## Project Workflow

The development process consists of the following steps:

1. **System Modeling and Controller Design**

   * A mathematical model of the self-balancing robot was developed in **MATLAB/Simulink**.
   * A **Full-State Feedback (FSB)** controller was designed using the **Linear Quadratic Regulator (LQR)** method to achieve stable balancing performance.

2. **Data Collection**

   * While the robot was controlled by the FSB controller, data were collected for machine learning.
   * The recorded features included:

     * Pitch angle (estimated using a complementary filter)
     * Angular velocity
     * FSB controller output (control signal)

3. **Model Training**

   * The collected dataset was imported into **Edge Impulse**.
   * A TinyML model was trained to approximate the behavior of the LQR-based controller.

4. **Deployment**

   * The trained model was converted to **TensorFlow Lite Micro**.
   * It was quantized to **8-bit integer precision** to reduce memory usage and computational complexity.
   * The final model was deployed and executed on the **Arduino Nano 33 BLE Sense** for real-time balance control.

## Technologies Used

* Arduino Nano 33 BLE Sense
* MATLAB / Simulink
* Linear Quadratic Regulator (LQR)
* Full-State Feedback (FSB) Control
* Edge Impulse
* TensorFlow Lite Micro
* Tiny Machine Learning (TinyML)

## Repository Structure

```text
├── Arduino/          # Arduino source code
├── MATLAB/           # MATLAB/Simulink models and scripts
├── Model/            # Trained TinyML model and TensorFlow Lite files
├── Data/             # Collected datasets 
└── README.md
```
## Requirements & Setup

Before running the Arduino code, the following setup steps are required:

1. **Install Required Library**

You must manually add the following library to the Arduino IDE:


``` ei-complimentary_98-arduino-1.0.6.zip ``` (located in the text ```model```/ folder)


To install it:

Open Arduino IDE

Go to Sketch → Include Library → Add .ZIP Library

Select the file from the ```model``` folder

2. **IMU Calibration**

IMU calibration was required to ensure accurate sensor readings for stable balancing performance.

The calibration procedure and code are included in the ```Arduino```/ folder.
Make sure to run the calibration script before uploading the main control code.

## Results

The trained TinyML model successfully reproduces the behavior of the LQR-based controller while being compact enough to run on the Arduino Nano 33 BLE Sense. Through 8-bit quantization and TensorFlow Lite Micro deployment, the model achieves efficient real-time inference suitable for embedded applications.

## Future Work

* Improve model accuracy through additional training data.
* Compare TinyML performance with classical control methods under disturbances.
* Optimize inference latency and power consumption.
* Explore more advanced neural network architectures for embedded control.

## License

This project is released under the MIT License. Feel free to use, modify, and distribute it for research and educational purposes.
