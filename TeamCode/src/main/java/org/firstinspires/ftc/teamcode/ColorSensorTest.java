package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class ColorSensorTest extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

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
