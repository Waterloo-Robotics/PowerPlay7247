package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class bingus extends LinearOpMode {
    public DcMotor motor1;
    Servo servo1;

    public void runOpMode() {
        motor1 = hardwareMap.dcMotor.get("cone");
        servo1 = hardwareMap.servo.get("fan");
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_x);
            if (gamepad1.b){
                servo1.setPosition(0.5);
            } else {
                servo1.setPosition(1);
            }
        }

    }

}
