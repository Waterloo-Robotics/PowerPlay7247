package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class REVMotorTest extends LinearOpMode {

    AttachmentControl attachmentControl = new AttachmentControl();
    TelemetryControl telemetryControl = new TelemetryControl();

    boolean isBPressed = false;
    public void runOpMode() {

        attachmentControl.attachmentInit(hardwareMap);
        telemetryControl.telemetryInit(telemetry);

        waitForStart();

        while (opModeIsActive()) {

//            attachmentControl.revEncoderTest(gamepad1.b, isBPressed);

            attachmentControl.revMotorTest(gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetryControl.telemetryUpdate(telemetry, "REV1 Revs", String.valueOf(AttachmentControl.REV_Motor1.getCurrentPosition() / 288));
            telemetryControl.telemetryUpdate(telemetry, "REV1 Speed", String.valueOf(AttachmentControl.REV_Motor1.getVelocity() / 288 * 60));
            telemetryControl.telemetryUpdate(telemetry, "REV2 Revs", String.valueOf(AttachmentControl.REV_Motor2.getCurrentPosition() / 288));
            telemetryControl.telemetryUpdate(telemetry, "REV2 Speed", String.valueOf(AttachmentControl.REV_Motor2.getVelocity() / 288 * 60));
            telemetryControl.update(telemetry);

        }

    }

}
