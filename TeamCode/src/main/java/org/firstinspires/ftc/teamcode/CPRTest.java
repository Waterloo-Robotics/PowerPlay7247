package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class CPRTest extends LinearOpMode {

    public DcMotor motor;

    TelemetryControl telemetryControl = new TelemetryControl();

    int motorPos;

    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("REV1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {

            motorPos = motor.getCurrentPosition();

            telemetryControl.telemetryUpdate(telemetry, "Motor Position", String.valueOf(motorPos));
            telemetryControl.update(telemetry);

        }

    }

}
