# rpi-pwm-fan-control

> This repository is a fork of [alexfukahori/rpi-pwm-fan-control](https://github.com/alexfukahori/rpi-pwm-fan-control).
> The original script was built on WiringPi-Python; this fork rewrites it on top of [pigpio](https://abyz.me.uk/rpi/pigpio/) and adds a systemd service, a linear fan curve and several reliability fixes.

## Motivation
This Python script was created for the purpose to control a PWM FAN Noctua NF-A4x20 5V, using Raspberry Pi 4B hardware features. The script uses pigpio as a dependency, it is used to drive the Hardware PWM at 25kHz as specified by Intel (c.f. “4-Wire Pulse Width Modulation (PWM) Controlled Fans”, Intel
Corporation September 2005, revision 1.3), and to read the FAN speed from the tachometer using a pin callback (interrupt on the falling edge). Using Hardware features uses less CPU to do the work, avoiding spending Raspberry Pi CPU resources only to control the FAN speed.

## Feature
- Use Hardware PWM pin (GPIO 12, 13, 18 or 19) to control the FAN
- Use a pin callback on the tachometer pin to read FAN speed
- Connect all the FAN 4pin directly to the Raspberry Pi pins (it is possible, because the internal pull-up resistor is enabled on the tachometer pin)
- Fan speed ramps linearly between `MIN_TEMP` and `MAX_TEMP`, is at full speed above `MAX_TEMP` and turns off below `OFF_TEMP`
- Turns the fan off and restores the original pin modes when stopped (Ctrl+C or `systemctl stop`)

## Dependencies
* [Python 3](https://www.python.org/downloads/) - The script interpreter
* [pigpio](https://abyz.me.uk/rpi/pigpio/) - The `pigpiod` daemon must be running; the script talks to it through the `pigpio` Python module

## Documentations
* [Noctua white paper](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf) - Noctua PWM specifications white paper
* [pigpio Python interface](https://abyz.me.uk/rpi/pigpio/python.html) - pigpio Python module documentation

## How to use
### Get repository
```shell
git clone git@github.com:MCFreddie777/rpi-pwm-fan-control.git
cd rpi-pwm-fan-control
pip3 install -r requirements.txt
```

### Start the pigpio daemon
The script needs `pigpiod` running, otherwise it exits immediately.

```shell
sudo apt install pigpio
sudo systemctl enable --now pigpiod
```

### Set GPIO pins
Edit the `PWM_PIN` and `TACH_PIN` values in [rpi-pwmfan.py](./rpi-pwmfan.py) to match the pins used by your Pi.
The temperature thresholds (`MAX_TEMP`, `MIN_TEMP`, `OFF_TEMP`) and fan speed limits (`RPM_MAX`, `RPM_MIN`) can be tuned in the same file.

### Run the script
```shell
python3 ./rpi-pwmfan.py
```

### Use the script as a background service
Please edit the `rpi-pwm-fan-control.service` file and replace the `absolute_path_to_this_repo` with your path.
E.g.: `ExecStart=python3 /home/pi/scripts/rpi-pwm-fan-control/rpi-pwmfan.py`

Then run following commands:

```shell
sudo cp rpi-pwm-fan-control.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable rpi-pwm-fan-control
```

## TODO List
* Accept suggestion! ;-)
