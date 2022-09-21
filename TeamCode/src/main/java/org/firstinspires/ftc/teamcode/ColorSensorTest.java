package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {

    AttachmentControl attachmentControl = new AttachmentControl();

    public void runOpMode() {

        attachmentControl.attachmentInit(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Color Sensor Red", AttachmentControl.color.red());
            telemetry.addData("Color Sensor Green", AttachmentControl.color.green());
            telemetry.addData("Color Sensor Blue", AttachmentControl.color.blue());
            telemetry.addData("Distance (cm)", ((DistanceSensor) AttachmentControl.color).getDistance(DistanceUnit.CM));
            telemetry.update();

        }

    }

}