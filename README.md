# rpi-pwm-fan-control

## Motivation
This Python script was created for the purpose to control a PWM FAN Noctua NF-A4x20 5V, using Raspberry Pi 4B hardware features. The script use WiringPi-Python as a dependecy, it is used to set Hardware PWM value clock as 25Khz that was specified by Intel (c.f. “4-Wire Pulse Width Modulation (PWM) Controlled Fans”, Intel
Corporation September 2005, revision 1.3), to read the FAN speed using the tachometer, it is read by using Hardware provided interruption pin.

## Dependencies
* [Python 3](https://www.python.org/download/releases/3.0/) - The scrpit interpreter
* [WiringPi-Python](https://github.com/WiringPi/WiringPi-Python) - Control Hardware features of Rasbberry Pi

## Documentations
* [Noctua white paper](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf) - Noctua PWM specifications white paper

## How to use
### Get repository
```sh
$ git clone git@github.com:alexfukahori/rpi-pwm-fan-control.git
$ cd rpi-pwm-fan-control
$ pip3 install -r requirements.txt
$ python3 ./rpi-pwmfan.py
```
