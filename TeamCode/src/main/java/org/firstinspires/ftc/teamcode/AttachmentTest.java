package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends LinearOpMode {


    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open);

        double shDir = 0;

        double elDir = 0;

        double wristDir = 0;

        boolean isBPushed = false;

        boolean claw = true;

        waitForStart();

        while (opModeIsActive()) {

            // TODO update these controls

            if (gamepad1.dpad_up) shDir = 1; else if (gamepad1.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad1.y) elDir = -1; else if (gamepad1.a) elDir = 1; else elDir = 0;

            if (gamepad1.dpad_left) wristDir = -1; else if (gamepad1.dpad_right) wristDir = 1; else wristDir = 0;

            if (gamepad1.b && !isBPushed) {

                claw = !claw;

                isBPushed = true;
            } else if (!gamepad1.b) {

                isBPushed = false;

            }

            attachmentControl.armManual(shDir, elDir, wristDir, claw, telemetryControl);

            telemetryControl.update();

        }

    }

}
