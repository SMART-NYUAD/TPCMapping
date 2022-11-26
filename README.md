# PTU-Control
Files for controlling FLIR Pan Tilt Unit

Usage: 

```
roslaunch ptu_control ptu_control.launch
```

Services: 

Setting pan and tilt (arguments v1, v2, e1, e2 are optional with default a value of 0)
```
rosrun ptu_control pan_tilt_client <pan> <tilt> <v1> <v2> <e1> <e2>
```
