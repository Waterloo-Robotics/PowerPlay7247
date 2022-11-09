package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends LinearOpMode {


    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

        double shDir = 0;

        double elDir = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) shDir = 1; else if (gamepad1.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad1.y) elDir = -1; else if (gamepad1.a) elDir = 1; else elDir = 0;

            attachmentControl.armManual(shDir, elDir, gamepad1.left_stick_x, gamepad1.b, gamepad1.x, telemetryControl);

            telemetryControl.update();

        }

    }

}
