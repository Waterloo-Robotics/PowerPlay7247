## NOTICE

This repository contains a customized version of the FTC SDK for the Power Play (2022-2023) competition season, customized by Team #7247 The H2O Loo Bots. Find the original [FTC SDK here](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

## Getting Started
If you are new to robotics or new to *FIRST* Tech Challenge, then you should consider reviewing the FTC Blocks Tutorial to get familiar with how to use the control system.

[FTC Blocks Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial)

[Android Studio Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Android-Studio-Tutorial)

# Release Notes

### These will be updated at some point

### And yes I'm aware I repeat over some dates I'll fix it eventually but not tonight

## 19.12.2022

* Removed encoder limit on wrist in armManualComp
* Started tests with utilising arm in autonomous
* Added functions to open and close claw

## 12.12.2022

* Updated automatic position setting of arm
* Added option inside AttachmentControl to mark an opMode as TeleOp
* Updated Autonomous
* Deleted unnecessary files
* Updated TelemetryControl
* Added subpackage "Tests" to org.firstinspires.ftc.teamcode and moved files into it
* Updated to FtcRobotController SDK 8.1.1

## 2022 IA Nuclear_Meet_2022-03-12

* Tweaks to odometry
* Tweaks to autonomous

## 05.12.2022

* Added second touch sensor to elbow
* Commented out buggy arm automation code from the Competition TeleOp opmode
* Added automatic method to set all three arm motors' positions, with the option to stall the opmode until it finishes
* Updated SDK to 8.1.0
* Updated Readme
* Further tweaks to automatic and manual arm code
* Added camera vision in TeleOp, might utilise later
* Had third competition, see 2022 IA Nuclear_Meet_2022-03-12

## 28.11.2022

* More tweaking of arm automation

## 2022 IA Nuclear_Meet_2022-19-11

* Minor adjustments to autonomous
* Minor adjustments to arm control

## 21.11.2022

* Added some rudimentary explanatory comments to code
* Added touch sensors to the arm
* Minor adjustments to H2OLooBots library initialisation
* Further work on arm automation
* More tweaks to arm control
* Added separate opmode for other side of field
* Had second competition, see 2022 IA Nuclear_Meet_2022-19-11

## 2022 IA Nuclear_Meet_2022-12-11

* Changed autonomous to not run into signal cone
* Changed TeleOp drivetrain function to have a speed multiplier
* Changed arm controls
* Minor autonomous tweaks

## 14.11.2022

* Started rough arm automation
* Improved arm manual functionality
* Added claw functionality
* Improved autonomous
* Improved arm initialisation
* Added separate functions for testing and competition for the arm
* Got rid of useless files
* Changed arm controls
* Had first competition, see 2022 IA Nuclear_Meet_2022-12-11

## 07.11.2022

* Updated Readme
* Added wrist functionality
* Changed the way that the H2OLooBots library was initialised
* Added wrist functionality

## 31.10.2022

* Finished initial odometry tuning
* Added shoulder and elbow functionality to the arm

## 24.10.2022

* Odometry tuning
* Testing of different motors and sensors

## 17.11.2022

* Attempts to automate arm movement in TeleOp

## 10.11.2022

* Updated TeleOp, AttachmentTest, and Autonomous to be competition ready

## 03.11.2022

* Developed first Autonomous OpMode
* Updated README.md (I haven't been very good at updating it sorry to the like 2 people watching this repository D:)

## 27.10.2022

* Further tuning of RoadRunner

## 20.10.2022

* Further adjustment to DriveTrain and AttachmentControl

## 13.10.2022

* Odometry tuning and some adjustments to DriveTrain, AttachmentControl, and TelemetryControl

## 06.10.2022

* Made TeleOp OpMode
* Copied [FTC 7247 Library](https://github.com/Waterloo-Robotics/FTC-H2OLoo-Quickstart) into the repository at [Teamcode/src/main/java/com/ftc/waterloo/h2oloobots](https://github.com/Waterloo-Robotics/PowerPlay7247/Teamcode/src/main/java/com/ftc/waterloo/h2oloobots)
* Copied [RoadRunner Odometry SDK](https://github.com/acmerobotics/roadrunner-quickstart) into the repository at [Teamcode/src/main/java/encoder/odo/ftc/rr](https://github.com/Waterloo-Robotics/PowerPlay7247/Teamcode/src/main/java/encoder/odo/ftc/rr)

## Initial Commit

* Cloned FTC SDK
