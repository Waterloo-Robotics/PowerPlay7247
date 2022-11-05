package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    double flpower, frpower, blpower, brpower;

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);


        waitForStart();

        while (opModeIsActive()) {

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetryControl.telemetryUpdate("Distance (Distance sensor)", String.valueOf(AttachmentControl.distance.getDistance(DistanceUnit.CM)));
            telemetryControl.telemetryUpdate("Distance (Color sensor)", String.valueOf(((DistanceSensor) AttachmentControl.color).getDistance(DistanceUnit.CM)));
            telemetryControl.telemetryUpdate("Red Input (Color sensor)", String.valueOf(AttachmentControl.color.red()));
            telemetryControl.telemetryUpdate("Green Input (Color sensor)", String.valueOf(AttachmentControl.color.green()));
            telemetryControl.telemetryUpdate("Blue Input (Color sensor)", String.valueOf(AttachmentControl.color.blue()));
            telemetryControl.motorTelemetryUpdate(flpower, frpower, blpower, brpower);
            telemetryControl.update();

        }

    }

}
