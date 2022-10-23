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

    public static DcMotorEx REV_Motor1;
    public static DcMotorEx REV_Motor2;

    public static double cpr = 288;

    public static double deg = 10;

    public void attachmentInit(HardwareMap hardwareMap) {

//        color = hardwareMap.colorSensor.get("color");
//        distance = (DistanceSensor) hardwareMap.get(DistanceSensor.class, "distance");
        REV_Motor1 = (DcMotorEx) hardwareMap.dcMotor.get("REV1");
        REV_Motor2 = (DcMotorEx) hardwareMap.dcMotor.get("REV2");
        REV_Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        REV_Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        REV_Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REV_Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REV_Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        REV_Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void revMotorTest(double motor1Input, double motor2Input) {

//        REV_Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        REV_Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        REV_Motor1.setVelocity(revToVelo(125) * motor1Input);
//        REV_Motor2.setVelocity(revToVelo(125) * motor2Input);

        REV_Motor1.setPower(1);
        REV_Motor2.setPower(1);

    }

    public void revEncoderTest(boolean button, boolean isButtonPressed) {

        if (button) {

            if (!isButtonPressed) {

                REV_Motor1.setTargetPosition((int) (REV_Motor1.getCurrentPosition() + cpr / 360 * deg));
                REV_Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                REV_Motor1.setVelocity(revToVelo(31.25));

                isButtonPressed = true;

            } else {};

        }

        if (REV_Motor1.getCurrentPosition() != REV_Motor1.getTargetPosition()) {

            REV_Motor1.setVelocity(revToVelo(62.5));

        } else {

            REV_Motor1.setVelocity(0);

        }



    }

    public double revToVelo(double rpm) {

        return rpm / 60 * cpr;

    }

}
