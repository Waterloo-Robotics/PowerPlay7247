package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends LinearOpMode {

    AttachmentControl attachmentControl = new AttachmentControl();
    TelemetryControl telemetryControl = new TelemetryControl();

    public void runOpMode() {

        attachmentControl.attachmentInit(hardwareMap);
        telemetryControl.telemetryInit(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            attachmentControl.setShoulderManual(-gamepad1.left_stick_y);
            attachmentControl.setElbowManual(gamepad1.left_stick_x);

            telemetryControl.telemetryUpdate(telemetry, "Shoulder Pos", String.valueOf(AttachmentControl.shoulder.getCurrentPosition()));
            telemetryControl.update(telemetry);

        }

    }

}
