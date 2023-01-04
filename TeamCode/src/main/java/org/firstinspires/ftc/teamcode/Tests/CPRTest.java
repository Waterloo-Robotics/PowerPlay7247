package org.firstinspires.ftc.teamcode.Tests;

import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class CPRTest extends LinearOpMode {

    public DcMotor motor;

    int motorPos;

    public void runOpMode() { // literally just reads encoder counts so I know which number to take off the website for counts per revolution

        motor = hardwareMap.dcMotor.get("REV1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            motorPos = motor.getCurrentPosition();

            telemetryControl.addData("Motor Position", String.valueOf(motorPos));
            telemetryControl.update();

        }

    }

}
