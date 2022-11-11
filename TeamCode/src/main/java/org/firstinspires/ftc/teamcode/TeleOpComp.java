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

        double shDir = 0;

        double elDir = 0;

        double wristDir = 0;

        boolean isAPushed = false;

        boolean pickUp = false;

        boolean up = false;

        boolean score = false;

        boolean claw = true;

        boolean isBPushed = false;

        boolean yPressed = false;

        AttachmentControl.claw.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) yPressed = true;

            if (gamepad2.dpad_up) shDir = 1; else if (gamepad2.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad2.y) elDir = -1; else if (gamepad2.a) elDir = 1; else elDir = 0;

            if (gamepad2.right_bumper) wristDir = -1; else if (gamepad2.left_bumper) wristDir = 1; else wristDir = 0;

            if (gamepad2.b && !isBPushed) {

                claw = !claw;

                isBPushed = true;
            } else if (!gamepad2.b) {

                isBPushed = false;

            }

            attachmentControl.armManualComp(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);


            // TODO Fix this
//            if (gamepad1.a && !isAPushed) {
//
//                if (pickUp && yPressed) {
//
//                    pickUp = false;
//                    score = true;
//
//                } else if (yPressed && !pickUp) {
//
//                    pickUp = true;
//                    score = false;
//
//                }
//
//                isAPushed = true;
//
//            } else if (!gamepad1.a) {
//
//                isAPushed = false;
//
//            }
//
//            attachmentControl.armAuto(pickUp, score, claw);

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetryControl.motorTelemetryUpdate(flpower, frpower, blpower, brpower);
            telemetryControl.update();

        }

    }

}
