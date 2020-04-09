### 1. Sine Wave Controller

```bash
roslaunch my_ur5_controller sine_wave_motion.launch
```

### 2. Cartesian Position Controller
```
roslaunch my_ur5_controller ur5_controller_trac_ik.launch
```

*commander_node* uses Joystick

Ps3 Controller:
* Left Joystic:  left: +x right: -x
* Left Joystic:  up  : +y down : -y
* Right Joystic: up  : +z down : -z