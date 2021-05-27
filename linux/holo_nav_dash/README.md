## ![logo](../../img/MARR_logo.png) [Microsoft Applied Robotics Research Library](https://special-giggle-b26bab5f.pages.github.io/)
### Open Source Samples for Service Robotics
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

# HoloNavDash
HoloLens Navigation Dashboard for ROS

## Launch Instructions
Run.
```
rosrun holo_nav_dash holo_nav_dash.py
```

With custom values.
```
rosrun holo_nav_dash holo_nav_dash.py _hnd_port:=8000
```

Publishing to a different topic (in this case `my_cmd_vel`).
```
rosrun holo_nav_dash holo_nav_dash.py cmd_vel:=my_cmd_vel
```

## Operations
```
Open your favorite webbrowser and navigate to http://localhost:8000
```

![HololensNavigation Dashboard UI](../../img/HololensNavigation_DashboardUI.png)