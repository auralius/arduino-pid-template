This repository contains a template for implementing a PID control in an Arduino Uno.

**Check the [this note](https://github.com/auralius/arduino-pid-template/blob/main/Notes%20on%20PID%20control%20with%20Arduino.pdf) on the control derivation.**

The PID is implemented as a periodic task for Timer 1. The PID runs constantly (hard realtime) at 1 kHz, while serial communication runs in the background and when the resource is available. 

As an example, we use a cheap N20 DC motor that is already equipped with a quadrature encoder.

<img src="https://github.com/auralius/arduino-pid-template/blob/main/N20.png" alt="Alt Text" style="width:30%; height:auto;">

The motor was purchased from:  

https://www.tokopedia.com/cncstorebandung/motor-dc-jga12-n20-dc-3-6v-high-torque-micro-dc-gearbox-jga12-n20-with-encoder-71-rpm-a1015

To drive the motor, we use an Arduino Motor Shiled rev 3. The complete system is as follows:


<img src="https://github.com/auralius/arduino-pid-template/blob/main/motor-control.png" alt="Alt Text" style="width:50%; height:auto;">

To avoid recompiling the codes every time we tune the controller, we propose the following input configuration:

<img src="https://github.com/auralius/arduino-pid-template/blob/main/tx.png" alt="Alt Text" style="width:50%; height:auto;">

To log and plot the motor data, we use CoolTerm and gnuplot. The logged data format is as follows:

<img src="https://github.com/auralius/arduino-pid-template/blob/main/rx.png" alt="Alt Text" style="width:50%; height:auto;">

The following GIF animation shows that the implemented controller works.

![](./demo1.gif)
