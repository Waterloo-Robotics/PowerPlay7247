package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends LinearOpMode {


    public void runOpMode() { // actively updated attachment test file, used to test new parts of the robot and control the arm without all the other dangerous parts of the robot active

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open, true);

        double wristDir = 0; // variable to combine the triggers to define wrist speed

        boolean isBPushed = false; // helper variable for toggle code

        boolean claw = true;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.right_trigger > 0.1) wristDir = -gamepad2.right_trigger; else if (gamepad2.left_trigger > 0.1) wristDir = gamepad2.left_trigger; else wristDir = 0;

            if (gamepad2.b && !isBPushed) { // toggle code, if you press B it switches between closed and open

                claw = !claw;

                isBPushed = true;
            } else if (!gamepad2.b) {

                isBPushed = false;

            }

//            attachmentControl.touchSensor();

            attachmentControl.armManual(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);

            attachmentControl.touchSensor();

            telemetryControl.update();

        }

    }

}
