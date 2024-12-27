This repository contains a template for implementing a PID control in an Arduino Uno.

**Check the [this note](https://github.com/auralius/arduino-pid-template/blob/main/Notes%20on%20PID%20control%20with%20Arduino.pdf) on the control derivation.**

In this branch, we will apply a PID control to a thermal system described in [this link](https://www.notion.so/Universitas-Pertamina-Temperature-Control-Device-02b5a889e17d4ee9ae5521881e55af0d).

<img src="https://github.com/auralius/arduino-pid-template/blob/thermal/heater-control.jpeg" alt="Alt Text" style="width:30%; height:auto;">

The thermal system has 2 heaters: **heater #1 is connected to PWM pin #9** and **heater #2 to PWM pin #10**. PWM #9 and #10 belong to **Timer 1**. Therefore, the PID control will be implemented as a periodic task of **Timer 2**. The PID runs constantly (hard realtime) at 1 kHz, while serial communication runs in the background and when the resource is available.


