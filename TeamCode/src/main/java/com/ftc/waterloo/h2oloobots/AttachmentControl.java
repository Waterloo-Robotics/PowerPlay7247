package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Holds All non-drivetrain programming (for the most part)

@Config
public class AttachmentControl {

    public static ColorSensor color;
    public static DistanceSensor distance;

    public static DcMotorEx shoulder;
    public static DcMotorEx elbow;

    public static Servo claw;

    public static DcMotorEx wrist;

    public static TouchSensor bottom;

    public static TouchSensor eltouch1;

    public static TouchSensor eltouch2;

    public static double cpr = 288;

    public static double deg = 10;

    double clawPos = 0;

    TelemetryControl telemetryControlLocal;

    public enum ServoPosition {

        open,
        closed

    }

    boolean TeleOp = false;

    // this initialises all attachments

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, ServoPosition position, boolean IsTeleOp) {

        TeleOp = IsTeleOp;

        shoulder = (DcMotorEx) hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        telemetryControlLocal = telemetryControl;

    }

    public void touchSensor() {

        telemetryControlLocal.telemetryUpdate("Elbow Touch 1", String.valueOf(eltouch1.isPressed()));
        telemetryControlLocal.telemetryUpdate("Elbow Touch 2", String.valueOf(eltouch2.isPressed()));
        telemetryControlLocal.telemetryUpdate("Bottom Touch", String.valueOf(bottom.isPressed()));

    }

    public void setShoulderManual(double speed) {

        shoulder.setPower(speed * 0.5);

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
        elbow.setPower(elbows);
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
        if (eltouch1.isPressed() && elbowSpeed < 0) elbows = 0; else if (elbow.getCurrentPosition() > 3450 && elbowSpeed > 0) elbows = 0; else if (eltouch2.isPressed() && elbowSpeed > 0) elbows = 0; else elbows = elbowSpeed;

        if (wrist.getCurrentPosition() < -150 && wristSpeed < 0) wristSpeed = 0; else if (wrist.getCurrentPosition() > 850 && wristSpeed > 0) wristSpeed = 0;

        shoulder.setPower(shoulders);
        elbow.setPower(elbows);
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

    public void setArmTargetPositions(int shoulderp, int elbowp, int wristp) {

        shoulder.setTargetPosition(shoulderp);
        elbow.setTargetPosition(elbowp);
        wrist.setTargetPosition(wristp);

    }

    public void setArmPositions(int shoulderp, int elbowp, int wristp, boolean WAIT) {

        shoulder.setTargetPosition(shoulderp);
        elbow.setTargetPosition(elbowp);
        wrist.setTargetPosition(wristp);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(1);
        elbow.setPower(1);
        wrist.setPower(1);

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

    public boolean reachedTargetPosition(DcMotorEx motor) {

        if (motor.getCurrentPosition() >= motor.getTargetPosition() - 10 && motor.getCurrentPosition() <= motor.getTargetPosition() + 10) return true; else return false;

    }

    boolean auto = false;
    double wrists = 0;
    // code to move the arm between 2 positions while maintaining manual control, currently code to move between the 2 positions does not work
    public void armAuto(boolean pickUp, boolean upButton, boolean servoToggle, double shoulderSpeed, double elbowSpeed, double wristSpeed) {

        if (pickUp) {

            shoulderpos = 0;
            elbowpos = 2440;
            auto = true;

        } else if (upButton) {

            shoulderpos = 4176;
            elbowpos = 3480;
            auto = true;

        } else {

            if (bottom.isPressed() && shoulderSpeed < 0) shoulderpos = shoulderpos; else shoulderpos += (shoulderSpeed * 53);
            if (eltouch1.isPressed() && elbowSpeed < 0) elbowpos = elbowpos; else if (bottom.isPressed() && elbowSpeed > 0 && elbow.getCurrentPosition() > 3140) elbowpos = elbowpos; else elbowpos += (elbowSpeed * 53);

        }

        if (shoulderpos <= 0 && !bottom.isPressed() && auto) {

            shoulderpos -= 5;

        } else if (shoulderpos <= 0 && bottom.isPressed() && (shoulderpos < shoulder.getCurrentPosition() - 10)) {

            shoulderpos = shoulder.getCurrentPosition();

        }

        if (auto && elbow.getCurrentPosition() <= elbow.getTargetPosition() + 10 && elbow.getCurrentPosition() >= elbow.getTargetPosition() - 10) {

            shoulder.setPower(1);

        } else if (elbow.getCurrentPosition() != elbow.getTargetPosition() && auto) {

            shoulder.setPower(0);

        } else if (elbow.getCurrentPosition() <= elbow.getTargetPosition() + 10 && elbow.getCurrentPosition() >= elbow.getTargetPosition() - 10 && shoulder.getCurrentPosition() <= shoulder.getTargetPosition() + 10 && shoulder.getCurrentPosition() >= shoulder.getTargetPosition() - 10) {

            auto = false;

        } else if (!auto) {

            shoulder.setPower(0.875);

        }

        shoulder.setTargetPosition(shoulderpos);
        elbow.setTargetPosition(elbowpos);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (wristSpeed < 0 && wrist.getCurrentPosition() < 0) wrists = 0; else if (wristSpeed > 0 && wrist.getCurrentPosition() > 725) wrists = 0; else wrists = wristSpeed;

        elbow.setPower(1);
        wrist.setPower(wrists * 0.75);

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

    public double revToVelo(double rpm) {

        return rpm / 60 * cpr;

    }

}
