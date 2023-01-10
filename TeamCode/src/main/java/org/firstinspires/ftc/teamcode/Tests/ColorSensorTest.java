package org.firstinspires.ftc.teamcode.Tests;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "ur mom is as green as string text")
public class ColorSensorTest extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open, false, false);

        waitForStart();

        while (opModeIsActive()) {

            attachmentControl.clawColorTelemetry();
            telemetryControl.update();

        }

    }

}
