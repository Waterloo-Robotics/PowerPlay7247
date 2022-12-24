package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@Autonomous(name = "ArmTestAuto")
public class ArmTestAuto extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false);

        waitForStart(); // code to test moving the arm autonomously, picking up the cones from the stack of 5 then scoring on the medium junction next to it

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        this.pickup(attachmentControl);

        attachmentControl.setArmPositions(0, -3443, 4, true);

        this.servo(attachmentControl, 1);

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        this.pickup(attachmentControl);

        attachmentControl.setArmPositions(0, -3432, 4, true);

        this.servo(attachmentControl, 1);

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        this.pickup(attachmentControl);

        attachmentControl.setArmPositions(0, -3608, 4, true);

        this.servo(attachmentControl, 1);

        this.score(attachmentControl);

        this.servo(attachmentControl, 0);

        attachmentControl.setArmPositions(0, -0, -0, true);

    }

    public void score(AttachmentControl attachmentControl) { // rudimentary scoring position (medium junction)

        attachmentControl.setArmPositions(4497, -1755, 4, true);

        attachmentControl.setArmPositions(4497, -1755, -801, true);

    }

    public void pickup(AttachmentControl attachmentControl) { // down position to prevent hitting the wall

        attachmentControl.setArmPositions(0, -2308, 4, true);

    }

    public void servo(AttachmentControl attachmentControl, double position) { // servo setposition function (yes I know there's a function for closing and opening the servo I forgot and am too lazy to fix it

        AttachmentControl.claw.setPosition(position);

        timer.reset();
        while (timer.seconds() < 0.375) {}

    }

}

