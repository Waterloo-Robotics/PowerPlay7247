package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class REVMotorTest extends LinearOpMode {

    AttachmentControl attachmentControl = new AttachmentControl();
    TelemetryControl telemetryControl = new TelemetryControl();

    boolean isBPressed = false;
    public void runOpMode() {

        attachmentControl.attachmentInit(hardwareMap);
        telemetryControl.telemetryInit(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            attachmentControl.revEncoderTest(gamepad1.b, isBPressed);

        }

    }

}
