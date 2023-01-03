package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Holds All non-drivetrain physical programming (except camera stuff)

@Config
public class AttachmentControl {

    // color sensor on claw
    public static ColorSensor clawColor;

    public static DistanceSensor distance;

    // shoulderHub is the shoulder motor closer to the REV hubs
    public static DcMotorEx shoulder;
    public static DcMotorEx shoulderHub;

    public static DcMotorEx elbow;

    public static DcMotorEx wrist;

    public static Servo claw;

    public static TouchSensor bottom;

    // eltouch1 is the touch sensor on the arm, eltouch2 is the touch sensor the elbow channel presses when it goes too far down
    public static TouchSensor eltouch1;
    public static TouchSensor eltouch2;
    public static TouchSensor wristTouch;

    // old stuff from testing REV core hex motors, useless now
    public static double cpr = 288;

    public static double deg = 10;

    double clawPos = 0;

    TelemetryControl telemetryControlLocal;

    public enum ServoPosition { // enum to define whether the servo is open or closed on initialisation, set by opmode running

        open,
        closed

    }

    boolean TeleOp = false;

    // I genuinely don't know why this exists
    double elSpeed = 1;

    // this initialises all attachments
    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, ServoPosition position, boolean IsTeleOp, boolean home) { // initialisation period

        TeleOp = IsTeleOp; // checks for teleop variable, useless for now but I'm sure I had a use in mind at a point

        shoulder = (DcMotorEx) hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderHub = (DcMotorEx) hardwareMap.dcMotor.get("shoulderHub");
        shoulderHub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderHub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderHub.setDirection(DcMotorSimple.Direction.REVERSE);

        elbow = (DcMotorEx) hardwareMap.dcMotor.get("elbow");
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = (DcMotorEx) hardwareMap.dcMotor.get("wrist");
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // did this because code was casting errors at a time, useless now (I think, don't remove just to be safe)
        shoulder.setTargetPosition(0);
        shoulderHub.setTargetPosition(0);
        elbow.setTargetPosition(0);
        wrist.setTargetPosition(0);

        // scales claw so we can use 0 and 1 always, and only have to change one number with hardware changes
        claw = hardwareMap.servo.get("claw");
        claw.scaleRange(0, 0.25);

        // determines if the claw is open or closed at initialisation
        switch (position) {

            case open:
                claw.setPosition(1);
            break;

            case closed:
                claw.setPosition(0);
            break;

        }

        eltouch1 = hardwareMap.touchSensor.get("eltouch1");
        eltouch2 = hardwareMap.touchSensor.get("eltouch2");
        bottom = hardwareMap.touchSensor.get("bottom");
        wristTouch = hardwareMap.touchSensor.get("wristTouch");

        // sets local telemetryControl variable to make life nice
        telemetryControlLocal = telemetryControl;

        clawColor = hardwareMap.colorSensor.get("clawColor");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        if (home) {
            // moves until touch sensor is pressed, then zeroes out motors

            while (!wristTouch.isPressed()) {

                wrist.setPower(0.6);

            }

            wrist.setPower(0);
            wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // oh yeah that's why that exists it's to make sure the arm doesn't kill itself
            // right
            while (!eltouch1.isPressed()) {

                if (eltouch2.isPressed()) elSpeed = -elSpeed;

                elbow.setPower(elSpeed);

            }

            elbow.setPower(0);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (!bottom.isPressed()) {

                shoulder.setPower(-1);
                shoulderHub.setPower(-1);

            }

            shoulder.setPower(0);
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderHub.setPower(0);
            shoulderHub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void distanceSensor() {

        telemetryControlLocal.telemetryUpdate("Distance", String.valueOf(distance.getDistance(DistanceUnit.MM)));

    }

    public void clawManual(boolean closeButton, boolean openButton) {

        if (closeButton) {

            claw.setPosition(claw.getPosition() + 0.001);

        } else if (openButton) {

            claw.setPosition(claw.getPosition() - 0.001);

        }

        telemetryControlLocal.telemetryUpdate("Claw Pos", String.valueOf(claw.getPosition()));

    }

    public void clawColorTelemetry() {

        telemetryControlLocal.telemetryUpdate("Red", String.valueOf(clawColor.red()));
        telemetryControlLocal.telemetryUpdate("Green", String.valueOf(clawColor.green()));
        telemetryControlLocal.telemetryUpdate("Blue", String.valueOf(clawColor.blue()));
        telemetryControlLocal.telemetryUpdate("Distance", String.valueOf(((DistanceSensor) clawColor).getDistance(DistanceUnit.CM)));

    }

    public void openClaw() { // code to open claw easily

        claw.setPosition(0);

    }

    public void closeClaw() { // code to close claw easily

        claw.setPosition(1);

    }

    public void touchSensor() { // returns whether or not all three touch sensors are pressed

        telemetryControlLocal.telemetryUpdate("Elbow Touch 1", String.valueOf(eltouch1.isPressed()));
        telemetryControlLocal.telemetryUpdate("Elbow Touch 2", String.valueOf(eltouch2.isPressed()));
        telemetryControlLocal.telemetryUpdate("Bottom Touch", String.valueOf(bottom.isPressed()));
        telemetryControlLocal.telemetryUpdate("Wrist Touch", String.valueOf(wristTouch.isPressed()));

    }

    double shoulders, elbows, wrists = 0;

    int shoulderpos, elbowpos, wristpos = 0;

    // moves the arm manually without the aid of encoder counts, only limits are touch sensors
    public void armManual(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen) {

        if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else if (eltouch2.isPressed() && shoulderSpeed > 0) shoulders = 0; else shoulders = shoulderSpeed * 0.75;
        if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed > 0) elbows = 0; else elbows = elbowSpeed;
        if (wristTouch.isPressed() && wristSpeed > 0) wrists = 0; else wrists = wristSpeed * 0.6;

        shoulder.setPower(shoulders);
        shoulderHub.setPower(shoulders);
        elbow.setPower(-elbows);
        wrist.setPower(wrists);

        if (servoOpen) {

            claw.setPosition(1);

        } else {

            claw.setPosition(0);

        }

        telemetryControlLocal.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));
    }

    // moves the arm, supposed to have encoder limits to prevent driver error however it doesn't quite work right
    public void armManualComp(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen, TelemetryControl telemetryControl) {

        if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else if (eltouch2.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
        if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed < 0) elbows = 0; else elbows = elbowSpeed;
        if (wristTouch.isPressed() && wristSpeed > 0) wrists = 0; else wrists = wristSpeed * 0.6;

        telemetryControl.telemetryUpdate("bottom", String.valueOf(bottom.isPressed()));
        telemetryControl.telemetryUpdate("eltouch1", String.valueOf(eltouch1.isPressed()));
        telemetryControl.telemetryUpdate("eltouch2", String.valueOf(eltouch2.isPressed()));

//        if (wrist.getCurrentPosition() < -150 && wristSpeed < 0) wristSpeed = 0; else if (wrist.getCurrentPosition() > 850 && wristSpeed > 0) wristSpeed = 0;

        shoulder.setPower(shoulders);
        shoulderHub.setPower(shoulders);
        elbow.setPower(-elbows);
        wrist.setPower(wrists);

        if (servoOpen) {

            claw.setPosition(1);

        } else {

            claw.setPosition(0);

        }

        telemetryControl.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControl.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControl.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControl.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));

    }

    boolean up = false;
    boolean drop = false;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dropTimer = new ElapsedTime();

    // simple command to set the target positions for the arm
    public void setArmTargetPositions(int shoulderp, int elbowp, int wristp) {

        shoulder.setTargetPosition(shoulderp);
        shoulderHub.setTargetPosition(shoulderp);
        elbow.setTargetPosition(elbowp);
        wrist.setTargetPosition(wristp);

    }

    // sets arm positions, with the option to wait until all motors reach their positions before continuing
    public void setArmPositions(int shoulderp, int elbowp, int wristp, boolean WAIT) {

        shoulder.setTargetPosition(shoulderp);
        shoulderHub.setTargetPosition(shoulderp);
        elbow.setTargetPosition(elbowp);
        wrist.setTargetPosition(wristp);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderHub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(1);
        shoulderHub.setPower(1);
        elbow.setPower(1);
        wrist.setPower(0.75);

        if (WAIT) {

            while (!reachedTargetPosition(shoulder) || !reachedTargetPosition(elbow) || !reachedTargetPosition(wrist)) {

                telemetryControlLocal.telemetryUpdate("Shoulder Pos", String.valueOf(shoulder.getCurrentPosition()));
                telemetryControlLocal.telemetryUpdate("Elbow Pos", String.valueOf(elbow.getCurrentPosition()));
                telemetryControlLocal.telemetryUpdate("Wrist Pos", String.valueOf(wrist.getCurrentPosition()));
                telemetryControlLocal.update();

            }

//            shoulder.setPower(0);
//            elbow.setPower(0);
//            wrist.setPower(0);
//
//            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    // calculation to see if the encoder position is within 10 counts of the target position, returns boolean
    public boolean reachedTargetPosition(DcMotorEx motor) {

        if (motor.getCurrentPosition() >= motor.getTargetPosition() - 10 && motor.getCurrentPosition() <= motor.getTargetPosition() + 10) return true; else return false;

    }

    boolean auto = false;

    public static boolean clawAuto = false;

    boolean isArmAtPosition = false;

    public void armCompWithAutomation(boolean driver1Arm, boolean autoEnabled, boolean pickUp, boolean middlePos, boolean upButton, boolean servoToggle, boolean servoButton, double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean leftWrist, boolean rightWrist) {

        if (autoEnabled) {

            if (driver1Arm) {

                shoulderpos = 0;
                elbowpos = 0;
                wristpos = 0;
                auto = true;

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

            if (middlePos) { // checks for pickup button, and if true sets automatic positions for a close to pickup position (to prevent hitting the wall)

                shoulderpos = 0;
                elbowpos = -2252;
                wristpos = -10;
                auto = true; // variable to keep track of if the button was recently pressed

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            } else if (upButton && elbow.getCurrentPosition() > -2888) { // checks for score button, and if true sets automatic positions for scoring

                shoulderpos = 3972;
                elbowpos = -2254;
                wristpos = -37;
                auto = true; // variable to keep track of if the button was recently pressed

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            } else if (leftWrist && !auto) {

                shoulderpos = shoulder.getCurrentPosition();
                elbowpos = elbow.getCurrentPosition();
                wristpos = -10;
                auto = true; // variable to keep track of if the button was recently pressed

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            } else if (rightWrist && !auto) {

                shoulderpos = shoulder.getCurrentPosition();
                elbowpos = elbow.getCurrentPosition();
                wristpos = -700;
                auto = true; // variable to keep track of if the button was recently pressed

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            } else if (pickUp && shoulder.getCurrentPosition() < 100) {

                shoulderpos = 0;
                elbowpos = -3722;
                wristpos = -10;
                claw.setPosition(1);
                auto = true; // variable to keep track of if the button was recently pressed

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

        }

        if (this.reachedTargetPosition(shoulder) && this.reachedTargetPosition(elbow) && this.reachedTargetPosition(wrist)) isArmAtPosition = true; else isArmAtPosition = false; // checks for if the arm is at position, and sets a boolean to reflect that value

        if (auto && !isArmAtPosition && autoEnabled) { // checks if autonomous is true, and if the arm hasn't reached its position, and if both of those are correct it continues with the automatic code

            if (clawTime.seconds() > 0.33 && clawTime.seconds() < 0.58) {

                claw.setPosition(0);

                clawAuto = true;

            }

            if (clawTime.seconds() > 0.58) {

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

        } else { // if the arm reaches its position, or if the autonomous arm variable is false, return manual control to the user

            auto = false;
            if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
            if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed > 0) elbows = 0; else elbows = -elbowSpeed;
            if (wristTouch.isPressed() && wristSpeed > 0) wrists = 0; else wrists = wristSpeed * 0.6;

            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shoulder.setPower(shoulders);
            shoulderHub.setPower(shoulders);
            elbow.setPower(elbows);
            wrist.setPower(wrists);

            if (servoButton) {

                clawAuto = false;

            }

            if (servoToggle && !clawAuto) {

                claw.setPosition(0);

            } else if (!clawAuto) {

                claw.setPosition(1);

            }

        }

        telemetryControlLocal.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));

    }

    // code to move the arm between 2 positions while maintaining manual control
    public void armCompWithAutomationAndColor(boolean driver1Arm, boolean autoEnabled, boolean colorSensorEnabled, boolean pickUp, boolean middlePos, boolean upButton, boolean servoToggle, boolean servoButton, double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean leftWrist, boolean rightWrist) {

        if (autoEnabled) {

            if (driver1Arm) {

                shoulderpos = 0;
                elbowpos = 0;
                wristpos = 0;
                auto = true;

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

            if (colorSensorEnabled) {

                if (middlePos && ((DistanceSensor) clawColor).getDistance(DistanceUnit.CM) > 4.5) { // checks for pickup button, and if true sets automatic positions for a close to pickup position (to prevent hitting the wall)

                    shoulderpos = 0;
                    elbowpos = -2252;
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (upButton && elbow.getCurrentPosition() > -2888 && ((DistanceSensor) clawColor).getDistance(DistanceUnit.CM) < 4.5) { // checks for score button, and if true sets automatic positions for scoring

                    shoulderpos = 3972;
                    elbowpos = -2254;
                    wristpos = -37;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (leftWrist && !auto) {

                    shoulderpos = shoulder.getCurrentPosition();
                    elbowpos = elbow.getCurrentPosition();
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (rightWrist && !auto) {

                    shoulderpos = shoulder.getCurrentPosition();
                    elbowpos = elbow.getCurrentPosition();
                    wristpos = -700;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (pickUp && shoulder.getCurrentPosition() < 100) {

                    shoulderpos = 0;
                    elbowpos = -3722;
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (((clawColor.red() > 100 || clawColor.blue() > 100)) && !auto && elbow.getCurrentPosition() < -2000 && shoulder.getCurrentPosition() < 654 && ((DistanceSensor) clawColor).getDistance(DistanceUnit.CM) < 4.5) {

                    shoulder.setPower(0);
                    elbow.setPower(0);
                    wrist.setPower(0);

                    shoulderpos = 0;
                    elbowpos = -2254;
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    clawTime.reset();

                    this.setArmTargetPositions(shoulderpos, elbowpos, wristpos);

                }

            } else {

                if (middlePos) { // checks for pickup button, and if true sets automatic positions for a close to pickup position (to prevent hitting the wall)

                    shoulderpos = 0;
                    elbowpos = -2252;
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (upButton && elbow.getCurrentPosition() > -2888) { // checks for score button, and if true sets automatic positions for scoring

                    shoulderpos = 3972;
                    elbowpos = -2254;
                    wristpos = -37;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (leftWrist && !auto) {

                    shoulderpos = shoulder.getCurrentPosition();
                    elbowpos = elbow.getCurrentPosition();
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (rightWrist && !auto) {

                    shoulderpos = shoulder.getCurrentPosition();
                    elbowpos = elbow.getCurrentPosition();
                    wristpos = -700;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                } else if (pickUp && shoulder.getCurrentPosition() < 100) {

                    shoulderpos = 0;
                    elbowpos = -3722;
                    wristpos = -10;
                    auto = true; // variable to keep track of if the button was recently pressed

                    this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

                }

            }

        }

        if (this.reachedTargetPosition(shoulder) && this.reachedTargetPosition(elbow) && this.reachedTargetPosition(wrist)) isArmAtPosition = true; else isArmAtPosition = false; // checks for if the arm is at position, and sets a boolean to reflect that value

        if (auto && !isArmAtPosition && autoEnabled) { // checks if autonomous is true, and if the arm hasn't reached its position, and if both of those are correct it continues with the automatic code

            if (clawTime.seconds() > 0.33 && clawTime.seconds() < 0.58) {

                claw.setPosition(0);

                clawAuto = true;

            }

            if (clawTime.seconds() > 0.58) {

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

        } else { // if the arm reaches its position, or if the autonomous arm variable is false, return manual control to the user

            auto = false;
            if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
            if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed > 0) elbows = 0; else elbows = -elbowSpeed;
            if (wristTouch.isPressed() && wristSpeed > 0) wrists = 0; else wrists = wristSpeed * 0.6;

            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shoulder.setPower(shoulders);
            shoulderHub.setPower(shoulders);
            elbow.setPower(elbows);
            wrist.setPower(wrists);

            if (servoButton) {

                clawAuto = false;

            }

            if (servoToggle && !clawAuto) {

                claw.setPosition(0);

            } else if (!clawAuto) {

                claw.setPosition(1);

            }

        }

        telemetryControlLocal.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));

    }

    ElapsedTime clawTime = new ElapsedTime();

    public void clawColorAutoTest(boolean servoToggle, double shoulderSpeed, double elbowSpeed, double wristSpeed) {

        if (((clawColor.red() > 500 || clawColor.blue() > 500)) && !auto && elbow.getCurrentPosition() < -2000 && shoulder.getCurrentPosition() < 654) {

            shoulder.setPower(0);
            elbow.setPower(0);
            wrist.setPower(0);

            shoulderpos = 0;
            elbowpos = -911;
            wristpos = 6;
            auto = true; // variable to keep track of if the button was recently pressed

            clawTime.reset();

            this.setArmTargetPositions(shoulderpos, elbowpos, wristpos);

        }

        if (this.reachedTargetPosition(shoulder) && this.reachedTargetPosition(elbow) && this.reachedTargetPosition(wrist)) isArmAtPosition = true; else isArmAtPosition = false; // checks for if the arm is at position, and sets a boolean to reflect that value

        if (auto && !isArmAtPosition) { // checks if autonomous is true, and if the arm hasn't reached its position, and if both of those are correct it continues with the automatic code

            if (clawTime.seconds() > 0.33) {

                claw.setPosition(0);

            }

            if (clawTime.seconds() > 0.58) {

                this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

            }

            servoToggle = false;

        } else { // if the arm reaches its position, or if the autonomous arm variable is false, return manual control to the user

            auto = false;
            if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
            if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed < 0) elbows = 0; else elbows = -elbowSpeed;
            if (wristTouch.isPressed() && wristSpeed > 0) wrists = 0; else wrists = wristSpeed * 0.6;

            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shoulder.setPower(shoulders);
            shoulderHub.setPower(shoulders);
            elbow.setPower(elbows);
            wrist.setPower(wrists);

            if (servoToggle) {

                claw.setPosition(1);

            } else {

                claw.setPosition(0);

            }

        }

        telemetryControlLocal.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("shoulder target pos", String.valueOf(shoulder.getTargetPosition()));
        telemetryControlLocal.telemetryUpdate("elbow target pos", String.valueOf(elbow.getTargetPosition()));
        telemetryControlLocal.telemetryUpdate("wrist target pos", String.valueOf(wrist.getTargetPosition()));
        telemetryControlLocal.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));
        telemetryControlLocal.telemetryUpdate("auto", String.valueOf(auto));
        telemetryControlLocal.telemetryUpdate("timer", String.valueOf(clawTime.seconds()));

    }

    // returns a calculation of rpm converted to DcMotorEx's velocity (calculated in encoder ticks per second)
    public double revToVelo(double rpm, double cpr) {

        return rpm / 60 * cpr;

    }

    // quick code to bring the arm to a scoring position, old and position likely needs updated
    public void armToScore() {

        this.setArmPositions(4188,-4337,-37,false);

    }

    // quick code to bring the arm to a pickup position, old and position likely needs updated
    public void armToCatch() {

        this.setArmPositions(0,-911,6,false);

    }

}
