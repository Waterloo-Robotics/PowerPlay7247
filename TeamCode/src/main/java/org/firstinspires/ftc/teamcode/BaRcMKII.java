package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.OdometryControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BaRcMKII extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        OdometryControl odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false, true);

        waitForStart();

        odometryControl.back(2.5, 0.125);
        odometryControl.strafeLeft(51.5, 0.125);
        odometryControl.turn(0.35, 0.125);
        odometryControl.forward(12.5, 0.125);

        attachmentControl.setArmPositions(3880, -2479, 0, true); //To Score
        attachmentControl.setArmPositions(3880, -2479, -764, true); //To Score
        this.servo(1); //Open
        attachmentControl.setArmPositions(0, -3192, -6, true);
        this.servo(0); //Closed
        attachmentControl.setArmPositions(3880, -2479, 0, true); //To Score
        attachmentControl.setArmPositions(3880, -2479, -764, true); //To Score
        this.servo(1); //Open
        attachmentControl.setArmPositions(0, 0, 0, true);


    }

    public void servo(double position) { // servo setposition function (yes I know there's a function for closing and opening the servo I forgot and am too lazy to fix it

        AttachmentControl.claw.setPosition(position);

        timer.reset();
        while (timer.seconds() < 0.375) {}

    }

}
