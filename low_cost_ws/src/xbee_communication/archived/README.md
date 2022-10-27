# XBee Communication
This package allows you to transfer Apriltags messages or SubTInfo (artifacts poses + robots poses) with XBee modules.

Note: Please use the spelling 'XBee' with capital X and B in any formal documents.

## Hardware Preparations

You should have at least two sets of the XBee modules as followings:

| Parts       | Model          | Where to Purchase  |
| Xbee Module | XXX            |                    |
| Arduino     | XXX            |                    |
| Others      |                |                    |

We also assume you have
* A: A laptop or workstation as base station, running Ubuntu system with Docker installed.
* B: A Respberry Pi or other embedded boards with Docker installed, or another computing unit like item A.
* C: One or more like item A or B for mesh.

## Installation

We provide Docker images for
* GPU/CPU laptop or workstation
```
$ docker pull XXX/YYY:tag
```

* RPi 3B:

```
$ docker pull XXX/YYY:tag
```
TODO: should we add the following ROS packages to docker build?
* apriltags_ros
* apriltags_ros_test


### Give XBee a static usb port /dev/xbee

Machine A and B (essentially all machines)

Do it on every machine you wanna use XBee
```
$ source set_xbee_port.sh
```

### Machine A:

On the machine sending messages

Terminal 1:
```
$ rosrun xbee_communication xbee_encoder
```
Terminal 2:

TODO: misc -> scripts?
TODO: confirm python version (ROS is still Pythin 2.7. but misc/operation.py should run on Python 3? Any python virtual environment needed?)
```
$ python3 misc/xbee_operation.py
```

### Machine B:

On the machine receiving messages

Terminal 1:
```
$ rosrun xbee_communication xbee_decoder
```
Terminal 2:
```
$ python3 misc/xbee_operation.py
```

## How to Run (Mesh)

TODO: should we add docker run/join commands?
TODO: this looks the same as one-to-one?

### Give XBee a static usb port /dev/xbee
Do it on every machine you wanna use XBee
```
$ source set_xbee_port.sh
```

### On the machine sending messages
```
$ rosrun xbee_communication xbee_encoder
```
another terminal
```
$ python3 misc/xbee_mesh_robot.py
```
### On the mesh nodes machines
```
$ python3 misc/xbee_mesh.py
```
### On the machine receiving messages
```
$ rosrun xbee_communication xbee_decoder
```
another terminal
```
$ python3 misc/xbee_mesh_base.py
```

## Topic info
### On machine sending messages

Subscribe:
```
# Apriltags Detection Array - type: AprilTagDetectionArray
topic:
/tag_detections

# SubT info (robot pose + artifact pose) - type: SubTInfo
topic:
/subt_info
```

TODO: add
```
$ rostopic list
```

### On machine receiving messages

Publish:
```
# Apriltags Detection Array - type: AprilTagDetectionArray
topic:
/apriltags_from_xbee

# SubT info (robot pose + artifact pose) - type: SubTInfo
topic:
/xBee_subt_info

# Artifact Pose - type: ArtifactPoseArray
topic:
/artifact_pose_list
```
