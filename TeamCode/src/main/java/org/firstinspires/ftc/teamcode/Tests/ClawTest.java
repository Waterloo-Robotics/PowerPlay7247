package org.firstinspires.ftc.teamcode.Tests;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class ClawTest extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, true, false);

        AttachmentControl.claw.scaleRange(0, 1);
        AttachmentControl.claw.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            attachmentControl.clawManual(gamepad1.a, gamepad1.b);
            telemetryControl.update();

        }

    }

}
