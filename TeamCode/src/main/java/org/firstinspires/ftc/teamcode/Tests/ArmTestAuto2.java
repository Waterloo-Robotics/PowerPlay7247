package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "ArmTestAuto2")
public class ArmTestAuto2 extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false);

        waitForStart();

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        this.pickup(attachmentControl);

        attachmentControl.setArmPositions(214, -3623, 22, true);

        this.servo(attachmentControl, 1);

        this.pickup(attachmentControl);

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        this.pickup(attachmentControl);

        attachmentControl.setArmPositions(139, -3549, 4, true);

        this.servo(attachmentControl, 1);

        this.pickup(attachmentControl);

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

//        this.pickup(attachmentControl);
//
//        attachmentControl.setArmPositions(0, -3608, 4, true);
//
//        this.servo(attachmentControl, 1);
//
//        this.score(attachmentControl);
//
//        this.servo(attachmentControl, 0);

        attachmentControl.setArmPositions(0, -0, -0, true);

    }

    public void score(AttachmentControl attachmentControl) {

        attachmentControl.setArmPositions(3929, -2078, 4, true);

        attachmentControl.setArmPositions(3929, -2078, -703, true);

    }

    public void pickup(AttachmentControl attachmentControl) {

        attachmentControl.setArmPositions(0, -2308, 4, true);

    }

    public void servo(AttachmentControl attachmentControl, double position) {

        AttachmentControl.claw.setPosition(position);

        timer.reset();
        while (timer.seconds() < 0.375) {}

    }

}

