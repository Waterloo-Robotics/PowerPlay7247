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
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

        double shDir = 0;

        double elDir = 0;

        double wristDir = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) shDir = 1; else if (gamepad1.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad1.y) elDir = -1; else if (gamepad1.a) elDir = 1; else elDir = 0;

            if (gamepad1.dpad_left) wristDir = -1; else if (gamepad1.dpad_right) wristDir = 1; else wristDir = 0;

            attachmentControl.armManual(shDir, elDir, wristDir, gamepad1.b, gamepad1.x, telemetryControl);

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetryControl.motorTelemetryUpdate(flpower, frpower, blpower, brpower);
            telemetryControl.update();

            // TODO Set Up auto arm - screenshots on DH

        }

    }

}
