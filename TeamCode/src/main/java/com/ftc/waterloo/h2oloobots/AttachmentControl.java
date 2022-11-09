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

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl) {

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
        claw.setPosition(0);

//        color = hardwareMap.colorSensor.get("color");

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

    public void armManual(double shoulderSpeed, double elbowSpeed, double wristSpeed, boolean clawBut1, boolean clawBut2, TelemetryControl telemetryControl) {

        shoulder.setPower(shoulderSpeed * 0.75);
        elbow.setPower(elbowSpeed);
        wrist.setPower(wristSpeed * 0.25);

        if (clawBut1) {

            clawPos += 0.01;

        } else if (clawBut2) {

            clawPos -= 0.01;

        }

        claw.setPosition(clawPos);

        telemetryControl.telemetryUpdate("shoulder pos", String.valueOf(shoulder.getCurrentPosition()));
        telemetryControl.telemetryUpdate("elbow pos", String.valueOf(elbow.getCurrentPosition()));
        telemetryControl.telemetryUpdate("wrist pos", String.valueOf(wrist.getCurrentPosition()));
        telemetryControl.telemetryUpdate("claw pos", String.valueOf(claw.getPosition()));

    }

    boolean up = false;
    boolean drop = false;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dropTimer = new ElapsedTime();

    public void armAuto(boolean pickUp, boolean upButton, boolean dropButton) {

        if (pickUp) {

            elbow.setTargetPosition(4568);
            wrist.setTargetPosition(16);
            claw.setPosition(0);

        } else if (upButton) {

            up = true;

            timer.reset();

            claw.setPosition(1);

        } else if (dropButton) {

            dropTimer.reset();
            drop = true;

            shoulder.setTargetPosition(4532);
            elbow.setTargetPosition(3575);
            wrist.setTargetPosition(149);

        }

        if (up && timer.seconds() >= 2) {

            shoulder.setTargetPosition(4532);
            elbow.setTargetPosition(2900);
            wrist.setTargetPosition(16);

        } else if (drop && dropTimer.seconds() >= 3) {

            claw.setPosition(0);

        }

        shoulder.setPower(0.875);
        elbow.setPower(1);
        wrist.setPower(0.5);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public double revToVelo(double rpm) {

        return rpm / 60 * cpr;

    }

}
