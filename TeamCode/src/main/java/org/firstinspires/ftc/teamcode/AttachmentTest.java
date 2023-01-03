package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class AttachmentTest extends LinearOpMode {

    public static boolean home = true;

    public void runOpMode() { // actively updated attachment test file, used to test new parts of the robot and control the arm without all the other dangerous parts of the robot active

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open, true, home);

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

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            attachmentControl.touchSensor();

//            attachmentControl.clawColorTelemetry();

            attachmentControl.distanceSensor();

            attachmentControl.armManual(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw);
//            attachmentControl.clawColorAutoTest(claw, -gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir);

//            attachmentControl.touchSensor();

            telemetryControl.update();

        }

    }

}
