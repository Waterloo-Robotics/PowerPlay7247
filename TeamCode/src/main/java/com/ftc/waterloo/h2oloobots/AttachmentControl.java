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

// Holds All non-drivetrain physical programming (except camera stuff)

@Config
public class AttachmentControl {

    // these currently don't have a use
    public static ColorSensor color;
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

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, ServoPosition position, boolean IsTeleOp) { // initialisation period

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
        claw.scaleRange(0.7, 1);

        // determines if the claw is open or closed at initialisation
        switch (position) {

            case open:
                claw.setPosition(0);
            break;

            case closed:
                claw.setPosition(1);
            break;

        }

        eltouch1 = hardwareMap.touchSensor.get("eltouch1");
        eltouch2 = hardwareMap.touchSensor.get("eltouch2");
        bottom = hardwareMap.touchSensor.get("bottom");

        // sets local telemetryControl variable to make life nice
        telemetryControlLocal = telemetryControl;

        // moves until touch sensor is pressed, then zeroes out motors
        while (!bottom.isPressed()) {

            shoulder.setPower(-1);
            shoulderHub.setPower(-1);

        }

        shoulder.setPower(0);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderHub.setPower(0);
        shoulderHub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderHub .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // oh yeah that's why that exists it's to make sure the arm doesn't kill itself
        // right
        while (!eltouch1.isPressed()) {

            if (eltouch2.isPressed()) elSpeed = -elSpeed;

            elbow.setPower(elSpeed);

        }

        elbow.setPower(0);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    }

    public void setShoulderManual(double speed) {

        shoulder.setPower(speed * 0.5);
        shoulderHub.setPower(speed * 0.5);

    }

    public void setElbowManual(double speed) {

        elbow.setPower(speed * 0.5);

    }

    public void setWristManual(double speed) {

        wrist.setPower(speed * 0.5);

    }

    double shoulders, elbows;

    int shoulderpos, elbowpos, wristpos = 0;

    public void armManual(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen, TelemetryControl telemetryControl) {

        if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else if (eltouch2.isPressed() && shoulderSpeed > 0) shoulders = 0; else shoulders = shoulderSpeed * 0.75;
        if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed > 0) elbows = 0; else elbows = elbowSpeed;

        shoulder.setPower(shoulders);
        shoulderHub.setPower(shoulders);
        elbow.setPower(-elbows);
        wrist.setPower(wristSpeed * 0.6);

        if (servoOpen) {

            claw.setPosition(0);

        } else {

            claw.setPosition(1);

        }

        telemetryControl.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControl.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControl.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControl.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));
    }

    // supposed to have encoder limits to prevent driver error, doesn't quite work right however
    public void armManualComp(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen, TelemetryControl telemetryControl) {

        if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else if (eltouch2.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
        if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed < 0) elbows = 0; else elbows = elbowSpeed;

        telemetryControl.telemetryUpdate("bottom", String.valueOf(bottom.isPressed()));
        telemetryControl.telemetryUpdate("eltouch1", String.valueOf(eltouch1.isPressed()));
        telemetryControl.telemetryUpdate("eltouch2", String.valueOf(eltouch2.isPressed()));

//        if (wrist.getCurrentPosition() < -150 && wristSpeed < 0) wristSpeed = 0; else if (wrist.getCurrentPosition() > 850 && wristSpeed > 0) wristSpeed = 0;

        shoulder.setPower(shoulders);
        shoulderHub.setPower(shoulders);
        elbow.setPower(-elbows);
        wrist.setPower(wristSpeed * 0.6);

        if (servoOpen) {

            claw.setPosition(0);

        } else {

            claw.setPosition(1);

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

    public void setArmTargetPositions(int shoulderp, int elbowp, int wristp) { // simple command to set the target positions for the arm

        shoulder.setTargetPosition(shoulderp);
        shoulderHub.setTargetPosition(shoulderp);
        elbow.setTargetPosition(elbowp);
        wrist.setTargetPosition(wristp);

    }

    public void setArmPositions(int shoulderp, int elbowp, int wristp, boolean WAIT) { // sets arm positions, with the option to wait until all motors reach their positions before continuing

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

    public boolean reachedTargetPosition(DcMotorEx motor) { // calculation to see if the encoder position is within 10 counts of the target position, returns boolean

        if (motor.getCurrentPosition() >= motor.getTargetPosition() - 10 && motor.getCurrentPosition() <= motor.getTargetPosition() + 10) return true; else return false;

    }

    boolean auto = false;
    double wrists = 0;

    boolean isArmAtPosition = false;
    // code to move the arm between 2 positions while maintaining manual control
    public void armAuto(boolean pickUp, boolean upButton, boolean servoToggle, double shoulderSpeed, double elbowSpeed, double wristSpeed) {

        if (pickUp) { // checks for pickup button, and if true sets automatic positions for a close to pickup position (to prevent hitting the wall)

            shoulderpos = 0;
            elbowpos = -911;
            wristpos = 6;
            auto = true; // variable to keep track of if the button was recently pressed

            this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

        } else if (upButton) { // checks for score button, and if true sets automatic positions for scoring

            shoulderpos = 4188;
            elbowpos = -4337;
            wristpos = -37;
            auto = true; // variable to keep track of if the button was recently pressed

            this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

        }

        if (this.reachedTargetPosition(shoulder) && this.reachedTargetPosition(elbow) && this.reachedTargetPosition(wrist)) isArmAtPosition = true; else isArmAtPosition = false; // checks for if the arm is at position, and sets a boolean to reflect that value

        if (auto && !isArmAtPosition) { // checks if autonomous is true, and if the arm hasn't reached its position, and if both of those are correct it continues with the automatic code

            this.setArmPositions(shoulderpos, elbowpos, wristpos, false);

        } else { // if the arm reaches its position, or if the autonomous arm variable is false, return manual control to the user

            auto = false;
            if (bottom.isPressed() && shoulderSpeed < 0) shoulders = 0; else shoulders = shoulderSpeed;
            if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed < 0) elbows = 0; else elbows = -elbowSpeed;
            wrists = wristSpeed;

            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderHub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shoulder.setPower(shoulders);
            shoulderHub.setPower(shoulders);
            elbow.setPower(elbows);
            wrist.setPower(wristSpeed);

        }

        if (servoToggle) {

            claw.setPosition(1);

        } else {

            claw.setPosition(0);

        }

        telemetryControlLocal.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControlLocal.telemetryUpdate("shoulder target pos", String.valueOf(shoulder.getTargetPosition()));
        telemetryControlLocal.telemetryUpdate("elbow target pos", String.valueOf(elbow.getTargetPosition()));
        telemetryControlLocal.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));
        telemetryControlLocal.telemetryUpdate("auto", String.valueOf(auto));
        telemetryControlLocal.telemetryUpdate("pickUp", String.valueOf(pickUp));
        telemetryControlLocal.telemetryUpdate("upButton", String.valueOf(upButton));

    }

    public double revToVelo(double rpm, double cpr) { // returns a calculation of rpm converted to DcMotorEx's velocity (calculated in encoder ticks per second)

        return rpm / 60 * cpr;

    }

    public void armToScore() { // quick code to bring the arm to a scoring position, old and position likely needs updated

        this.setArmPositions(4188,-4337,-37,false);

    }

    public void armToCatch() { // quick code to bring the arm to a pickup position, old and position likely needs updated

        this.setArmPositions(0,-911,6,false);

    }

}
