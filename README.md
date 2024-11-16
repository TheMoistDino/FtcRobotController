## Welcome!
This GitHub repo includes the source code used in 6373 SuperBots competition robot for the INTO THE DEEP (2024-2025) season.

## Robot Controls
### Gamepad 1
- Left and Right Joysticks = Movement  (left for translational, right for rotational)
- Right Bumper = Intake Claw
- Left Bumper = Bucket
- Right Trigger = Lift Up
- Left Trigger = Lift Down
- D-Pad Up = Arm Forward
- D-Pad Down = Arm Backward

### Gamepad 2
- Start Button = Toggle Robot-Oriented to Field-Oriented
- Right Bumper = Slow Driving
- Left Bumper = Slow Lift
- A = Slow Arm
- B = Toggle liftDebug mode
- XBack = Reset lift position to zero

## Change Log
### Meet 1 (11/16, 7:00)

### 4 Entity/Robot MeepMeep Simulator (11/10, 22:00)
* Added & adjusted paths in MeepMeepTesting.java

### New AUTO programs: AutoMeetOneL, AutoMeetOneLPush, AutoMeetOneR (11/10, 12:00)
* Implemented autonomous paths + actions using Road Runner
  * Two variations for the left paths
    * Score 3 in high basket (which is Left Baskets)
      * 27pt AUTO + 24 TELEOP = 51pt total
    * Score 1 in high basket + 2 in net zone (which is Left Push)
      * 13pt AUTO + 10 TELEOP = 23pt total
  * One variation for right path
    * Push 2 to the observation zone
      * 3pt AUTO Park
  * All park (Left in ASCENT zone, Right in Observation Zone)

### MeepMeep + Road Runner for AUTO (11/9, 23:30)
* Installed MeepMeep for RR 1.0.x
* Created predicted paths for AUTO for the left and right starting positions
  * Implemented in AutoMeetOneL.java
  * Not yet in AutoMeetOneR.java
* Other small changes in misc files

### Reorganized Variables + Added ArmToPosition Method (11/9, 14:00)
#### MotorControl.java + related classes
* Renamed armDirection variables to 'forward' and 'backward'
* Added ArmToPosition method

### Tuned & Adjusted Values (11/7, 19:00)
#### servoTuner.java, ServoControl.java, motorTuner.java
* Adjusted variable names and some values
#### MotorControl.java
* Tuned lift motor and set min and max ranges
#### TeleOp.java
* Added 2 functions to Gamepad 2 (A and X)
  * Allow the lift to move beyond incorrectly set program limits

### Added cameraColorTuner.java (11/5, 06:30)
* Performs color masking to specific ranges

### Included Gamepad controls + Servo Optimizations (11/3, 21:00)
#### ServoControl.java
* Changed Servo types to ServoImplEx
  * Improved control of servos to prevent movement between AUTO and TELEOP periods
  * Created StartServos and StopServos methods
* Increased range of servo's PWM range to utilize the full 270 degrees of the heavy-duty servos
#### AutoMeetOne.java and AutoMeetZero.java
* Disabled AutoMeetZero
* Created AutoMeetOne
#### TeleOp.java
* Added respective StartServos and StopServos methods

### Bug Fixes (11/1, 20:00)
#### TeleOp.java
* Updated Display Name to "Meet 1 TeleOp"
* Fixed Lift controls
#### MotorControl.java
* Adjusted max_armPower values & inputs
* Adjusted StopAndReturnLift method to make lift re-enter limits
#### ServoControl.java
* Adjusted closed and open claw positions
#### MecanumDrive.java
* Adjusted initialization functions

### Merged branch "test-002" into "test-001" (10/25, 21:00)
#### MotorControl.java
* Re-ordered variable initialization
* Rewrote "MoveArm" and "MoveLift" to be cleaner
* Created a "LiftToPosition" method for Auto and TeleOp (not implemented)

### New branch: test-002 (10/25, 19:00)
#### TeleOp.java
* Renamed TeleOpMeetZero to TeleOp
* Renamed SPEED_MULTIPLIER variables in TeleOp
* Added an operating range for the lift (in MotorControl, TeleOp)
#### HolonomicDrive.java
* Added a driving orientation mode button
* Changed IMU units from DEGREES to RADIANS
#### SampleTeleOp.java
* Disabled SampleTeleOp from showing in the Driver Hub menu

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.
