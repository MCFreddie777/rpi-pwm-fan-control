# rpi-pwm-fan-control

## Motivation
This Python script was created for the purpose to control a PWM FAN Noctua NF-A4x20 5V, using Raspberry Pi 4B hardware features. The script uses WiringPi-Python as a dependecy, it is used to set Hardware PWM value clock as 25Khz that was specified by Intel (c.f. “4-Wire Pulse Width Modulation (PWM) Controlled Fans”, Intel
Corporation September 2005, revision 1.3), to read the FAN speed using the tachometer, it is read by using Hardware provided interruption pin. Using Hardware feature use less CPU to execute the work, avoiding to use Raspberry Pi CPU resource only to control the FAN speed.

## Feature
- Use PWM pin to control the FAN
- Use Interruption pin to read FAN speed
- Connect all the FAN 4pin directly to the Raspberry Pi pins (it is possible, because I'm using internal pull-up resistor)
- CPU usage stay between 1% and 2%, because it needs to process the tachometer interruption routine

## Dependencies
* [Python 3](https://www.python.org/download/releases/3.0/) - The script interpreter

## Documentations
* [Noctua white paper](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf) - Noctua PWM specifications white paper

## How to use
### Get repository
```shell
git clone git@github.com:alexfukahori/rpi-pwm-fan-control.git
cd rpi-pwm-fan-control
pip3 install -r requirements.txt
```

### Set GPIO pins
Edit the `PWM_PIN` and `TACH_PIN` values in [rpi-pwmfan.py](./rpi-pwmfan.py) to match the pins used by your Pi.

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
