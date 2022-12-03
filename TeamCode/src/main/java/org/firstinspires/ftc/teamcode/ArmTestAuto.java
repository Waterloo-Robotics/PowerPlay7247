package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;

@Config
@Autonomous(name = "ArmTestAuto")
public class ArmTestAuto extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed);

        waitForStart();

        attachmentControl.setArmPositions(1800, 3000, 624, true);

        AttachmentControl.claw.setPosition(0);

        attachmentControl.setArmPositions(0, 0, 0, true);

    }
}

