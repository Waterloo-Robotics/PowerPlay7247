package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends LinearOpMode {


    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open, true);

        double shDir = 0;

        double elDir = 0;

        double wristDir = 0;

        boolean isBPushed = false;

        boolean claw = true;

        waitForStart();

        while (opModeIsActive()) {

            // TODO update these controls

            if (gamepad2.dpad_up) shDir = 1; else if (gamepad2.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad2.y) elDir = -1; else if (gamepad2.a) elDir = 1; else elDir = 0;

            if (gamepad2.right_trigger > 0.1) wristDir = -gamepad2.right_trigger; else if (gamepad2.left_trigger > 0.1) wristDir = gamepad2.left_trigger; else wristDir = 0;

            if (gamepad2.b && !isBPushed) {

                claw = !claw;

                isBPushed = true;
            } else if (!gamepad2.b) {

                isBPushed = false;

            }

//            attachmentControl.touchSensor();

            attachmentControl.armManual(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);

            attachmentControl.touchSensor();

//            attachmentControl.armManual(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);

            telemetryControl.update();

        }

    }

}
