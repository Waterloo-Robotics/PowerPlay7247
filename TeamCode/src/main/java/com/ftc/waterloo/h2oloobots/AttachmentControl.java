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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AttachmentControl {

    public static ColorSensor color;
    public static DistanceSensor distance;

    public static DcMotorEx shoulder;
    public static DcMotorEx elbow;

    public static Servo claw;

    public static DcMotorEx wrist;

    public static double cpr = 288;

    public static double deg = 10;

    double clawPos = 0;

    public enum ServoPosition {

        open,
        closed

    }

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, ServoPosition position) {

        shoulder = (DcMotorEx) hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow = (DcMotorEx) hardwareMap.dcMotor.get("elbow");
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = (DcMotorEx) hardwareMap.dcMotor.get("wrist");
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder.setTargetPosition(0);
        elbow.setTargetPosition(0);
        wrist.setTargetPosition(0);

        claw = hardwareMap.servo.get("claw");
        claw.scaleRange(0.7, 1);

        switch (position) {

            case open:
                claw.setPosition(0);
            break;

            case closed:
                claw.setPosition(1);
            break;

        }

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

    public void armManual(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen, TelemetryControl telemetryControl) {

        shoulder.setPower(shoulderSpeed * 0.75);
        elbow.setPower(elbowSpeed);
        wrist.setPower(wristSpeed * 0.25);

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

    public void armManualComp(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean servoOpen, TelemetryControl telemetryControl) {

        if (shoulder.getCurrentPosition() > -300) shoulder.setPower(shoulderSpeed * 0.75);
        if (elbow.getCurrentPosition() > -100) elbow.setPower(elbowSpeed);
        wrist.setPower(wristSpeed * 0.25);

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

    public void armAuto(boolean pickUp, boolean upButton, boolean servoToggle) {

        if (pickUp) {

            shoulder.setTargetPosition(0);
            elbow.setTargetPosition(-2454);
            wrist.setTargetPosition(-3);

        } else if (upButton) {
            shoulder.setTargetPosition(4493);
            elbow.setTargetPosition(-3459);
            wrist.setTargetPosition(140);

        }

        shoulder.setPower(0.75);
        elbow.setPower(1);
        wrist.setPower(0.5);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (servoToggle) {

            if (claw.getPosition() == 0) {

                claw.setPosition(1);

            } else if (claw.getPosition() == 1) {

                claw.setPosition(0);

            }

        }

    }

    public double revToVelo(double rpm) {

        return rpm / 60 * cpr;

    }

}
