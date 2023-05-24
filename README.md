# Inverted pendulum control

This repository contains algorithm of energy-based controller for inverted pendulum.

In `main.c` several parameters that depend on the physical system are listed:

- `g = 9.81` - gravity [m/s^2]
- `l = 0.617` - length of pendulum [m]
- `m_c = 0.355` - mass of cart [kg]
- `m_p = 0.216` - mass of pendulum [kg]
- `alpha = 10` - learining rate (> 0)
- `dt = 0.01` - step of discretization [s]

Positions of cart and pendulum are measured via encoders. To convert obtained ticks into position of cart and angle of pendulum, the following coefficients are listed:

- `cart_ticks_to_m = 0.05 / 1024` - converts ticks to cart position [m]
- `pend_ticks_to_rad = M_PI / 1024` - converts ticks to pendulum angle [rad]

Current values are measured and calculated for the existing physical system. They should be changed for different systems.

In this project STM32F407 microcontroller was used as a computational unit. To upload the code to controller you should follow these steps:

**Preliminaries**

1. Download STM32CubeIDE
2. Clone the code to your system

**Steps**

1. Open STM32CubeIDE
2. Go to `File -> Open Projects from File System`
3. Press `Directory...` and choose the project folder, then press `Open`
4. Press `Finish`
5. Connect the microcontroller to your PC
6. Press `Run`

## NOTE

These steps were tested on Ubuntu 22.04 system, therefore they may have differences in other systems