package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    AttachmentControl attachmentControl = new AttachmentControl();

    public void runOpMode() {

        attachmentControl.attachmentInit(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Distance", AttachmentControl.distance.getDistance(DistanceUnit.CM));
            telemetry.update();

        }

    }

}
