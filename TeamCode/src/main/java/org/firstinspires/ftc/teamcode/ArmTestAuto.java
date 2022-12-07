package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "ArmTestAuto")
public class ArmTestAuto extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        attachmentControl.setArmPositions(1800, 3000, 624, true);

        timer.reset();
        while (timer.seconds() < 1) {}

        AttachmentControl.claw.setPosition(0);

        timer.reset();
        while (timer.seconds() < 1) {}

        attachmentControl.setArmPositions(0, 0, 0, true);

    }
}

