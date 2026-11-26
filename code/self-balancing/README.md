# Self-balancing robot firmware

Firmware for a two-wheeled self-balancing robot: ESP32 NodeMCU-32S, TB6612FNG driver,
two JGA25-370 gear motors with quadrature encoders, MPU6050 IMU.
Arduino-ESP32 core 3.x, no external library.

Everything is tuned from a web page served by the robot over its own Wi-Fi access point.

## Control law

```
u = Kp*e + Kd*w + Kv*v + Kx*x        (200 Hz)
```

| Term | Meaning |
|---|---|
| `e` | tilt error, angle minus the balance angle (deg) |
| `w` | tilt rate from the gyro (deg/s) |
| `v` | mean wheel speed from the encoders (pulses/s) |
| `x` | distance travelled since arming (pulses) |

The angle comes from a complementary filter on the MPU6050. A PD on the angle alone leaves a slow
unstable mode (the robot drifts away in 2 to 4 s), which is why wheel speed and position are fed back too.

## How the gains are found

1. **Motor identification.** Wheels in the air, three PWM steps. A line fitted to the encoder
   position gives the steady speed and the time constant of each step, then the motor gain `K_m`,
   the dead zone `u0` and the time constant `tau`.
2. **LQR.** The page builds a 4-state model (position, wheel speed, angle, tilt rate) from `K_m`, `tau`,
   the wheel radius and the height of the centre of mass, discretises it at the loop period and solves
   the discrete Riccati equation. It then simulates a 2 degree initial tilt and warns if the command would saturate.
3. **Autotune on the real robot.** Twiddle (coordinate descent) on the four gains. Each trial is scored by
   `J = e^2 + wRate*w^2 + wV*v^2 + wX*x^2 + saturation penalty`, measured over two halves so that
   an oscillation that grows is penalised. A fall costs a large fixed amount.
4. **Validation.** A long final trial with the best gains. If the robot falls or the oscillation grows,
   the starting gains are restored.

## Other features

- Continuous dead-zone compensation (no relay-like jump around zero)
- Fall detection and automatic re-arming once the robot is upright again
- Motor and encoder tests, direction and swap settings from the page
- All settings saved to flash; firmware update over Wi-Fi (Arduino OTA or upload from the page)

## Wiring

| Signal | GPIO |
|---|---|
| TB6612 PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| TB6612 PWMB / BIN1 / BIN2 | 33 / 32 / 13 |
| TB6612 STBY | 4 |
| Encoder 1 A / B | 16 / 17 |
| Encoder 2 A / B | 18 / 19 |
| MPU6050 SDA / SCL | 21 / 22 |

## First start

Set your own password in `AP_PASS` and `OTA_PASS`, flash over USB, keep the robot still for one second
at power-up (gyro calibration), join the `Robot-Bal` network and open `http://192.168.4.1`.
The steps are listed at the top of the sketch (comments and web page are in French).
