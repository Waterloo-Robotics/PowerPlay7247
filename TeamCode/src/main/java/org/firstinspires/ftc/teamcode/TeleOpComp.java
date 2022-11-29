package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class TeleOpComp extends LinearOpMode {
    double flpower, frpower, blpower, brpower;
    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open);

        // Declaring variables for joystick controls (greyed out ones are no longer used)

        double shDir = 0; // shoulder direction - rotates up (positive numbers) or rotates down (negative numbers)

        double elDir = 0; // elbow direction- rotates down and extends out (positive numbers) or rotates up and folds in (negative numbers)

        double wristDir = 0; // wrist direction - rotate clockwise (positive numbers) or counterclockwise . View is from the front of the motors.

        boolean isAPushed = false; // automatic pickup position for arm and elbow

        boolean pickUp = false;

        boolean up = false;

        boolean score = false;

        boolean claw = true;

        boolean isBPushed = false; // toggle for opening/closing the claw. Holding the B button just makes the claw close.

        boolean yPressed = false;

        double speedMul = 0.55;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) speedMul = 1; else speedMul = 0.55;

//            if (gamepad1.y) yPressed = true;
//
//            if (gamepad2.dpad_up) shDir = 1; else if (gamepad2.dpad_down) shDir = -1; else shDir = 0;
//
//            if (gamepad2.y) elDir = -1; else if (gamepad2.a) elDir = 1; else elDir = 0;
//
//            if (gamepad2.right_bumper/* && AttachmentControl.wrist.getCurrentPosition() > 0*/) wristDir = -1; else if (gamepad2.left_bumper/* && AttachmentControl.wrist.getCurrentPosition() < 200*/) wristDir = 1; else wristDir = 0;
//
//            if (gamepad2.b && !isBPushed) {
//
//                claw = !claw;
//
//                isBPushed = true;
//            } else if (!gamepad2.b) {
//
//                isBPushed = false;
//
//            }
//
//
//            // TODO Fix this
//            if (gamepad2.a) {
//
//                pickUp = true;
//                score = false;
//                up = true;
//
//            } else if (gamepad2.y) {
//
//                pickUp = false;
//                score = true;
//                up = true;
//
//            } else {
//
//                pickUp = false;
//                score = false;
//
//            }

//
//            attachmentControl.armAuto(pickUp, score, claw);

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

//            attachmentControl.armAuto(pickUp, score, claw, -gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir);

            pickUp = false;
            score = false;

            attachmentControl.armManualComp(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speedMul);
            telemetryControl.update();

        }

    }

}
