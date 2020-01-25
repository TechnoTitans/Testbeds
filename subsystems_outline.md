Subsystems

Climbing
- Deploy
- Retract (?)

Drive Train
- Move
- MoveForward
- MoveBackward
- DrivePath
* (Not a command) safety (avoidance)

Control Panel Controller
- Extend
- Retract
- Rotate
- RotateToPosition
	- RGB sensor
- RotateNum
	- Encoder

* interface for knowing which color is required

Turret
- Rotate about 2 axes
 - 
- Shoot
 inputs:
 - distance: vision, ultra-sonic distance, time of flight
 outputs:
 - power
 - flywheel speed  

Ball Feeder
- Intake
- Out-take

* user interface: ball count


3 different stages: auto, teleop, end_game