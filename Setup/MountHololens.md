## ![logo](../img/MARR_logo.png) [Microsoft Applied Robotics Research Library](https://special-giggle-b26bab5f.pages.github.io/)
### Open Source Samples for Service Robotics
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

# HoloLens Mounting and Pepper Configuration Instructions

## HoloLens Device Portal

Enable Device Portal on the HoloLens device in `Settings->For developers`

Navigate to the HoloLens IP using your web browser and set up a user/password.

### Certificate Installation
Navigate to the HoloLens IP using your web browser and download the HoloLens certificate. Convert and install it:

```
$ sudo apt-get install ca-certificates -y
$ openssl x509 -outform der -in certificate.pem -out certificate.crt
$ sudo cp certificate.crt /usr/local/share/ca-certificates
$ sudo update-ca-certificates
```

### Change or disable HoloLens sleep settings
Navigate to the HoloLens IP using your web browser and log in with the user/pwd set above. In System->Preferences set sleep settings. To disable, use the browser's inspect feature and add `0` as a list option, then select `0`. Note you will need to do this twice if you want to disable sleep for both `battery` and `plugged-in` settings.

## Pepper Configuration

Pepper [qicli](http://doc.aldebaran.com/2-5/dev/libqi/guide/qicli.html) commands can be issued via the Choregraphe application or directly using a secure shell.

### Start Pepper with autonomous life disabled

Disable Pepper autonomous life mode using Choregraph:

- Connect to Pepper using Choregraph.
- Click on blue heart icon in upper right corner.
- Wake Pepper up by clicking on sunshine icon in upper right corner
 
Disable Pepper autonomous life mode using `ssh`:

```
$ ssh nao@<pepper IP>
  > nao stop
  > naoqi-bin --disable-life
```

Connect to Pepper again and:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.wakeUp
```

To make Pepper go to sleep:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.rest
```

To shut down Pepper:
```
$ ssh nao@<pepper IP>
  > sudo shutdown -h now
```

To get the joint names for the body or a chain:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.getBodyNames "Body"
```

To view Pepper's current joint state:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.getSummary
```

To change Pepper's head pitch:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3
```

The `setAngles` call is a non-blocking call. Parameters:
- `names` – The name or names of joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
- `angles` – One or more angles in radians
- `fractionMaxSpeed` – The fraction of maximum speed to use

Valid Pepper joint names can be found [here](http://doc.aldebaran.com/2-5/family/romeo/joints_romeo.html?highlight=joint) and [here](https://developer.softbankrobotics.com/nao6/nao-documentation/nao-developer-guide/kinematics-data/effector-chain-definitions#nao-chains),
or call `getBodyNames` for a complete list.

### Pepper RViz configuration file

location: `/opt/ros/melodic/share/naoqi_driver/share/pepper.rviz`


### Miscellaneous

Enumerate network interfaces on Ubuntu:

```
ip l show
ip a show eno1
```