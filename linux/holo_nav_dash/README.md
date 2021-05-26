# HoloNavDash
HoloLens Navigation Dashboard for ROS

# Launch
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

# Usage
```
Open your favorite webbrowser and navigate to http://localhost:8000
```

