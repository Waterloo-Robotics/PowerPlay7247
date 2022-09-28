package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    double flpower, frpower, blpower, brpower;

    DriveTrain driveTrain = new DriveTrain();
    AttachmentControl attachmentControl = new AttachmentControl();
    TelemetryControl telemetryControl = new TelemetryControl();

    public void runOpMode() {

        driveTrain.FourMotorInit(hardwareMap);
        attachmentControl.attachmentInit(hardwareMap);
        telemetryControl.telemetryInit(telemetry);


        waitForStart();

        while (opModeIsActive()) {

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, attachmentControl);

            telemetryControl.telemetryUpdate(telemetry, "Distance (Distance sensor)", String.valueOf(AttachmentControl.distance.getDistance(DistanceUnit.CM)));
            telemetryControl.telemetryUpdate(telemetry, "Distance (Color sensor)", String.valueOf(((DistanceSensor) AttachmentControl.color).getDistance(DistanceUnit.CM)));
            telemetryControl.telemetryUpdate(telemetry, "Red Input (Color sensor)", String.valueOf(AttachmentControl.color.red()));
            telemetryControl.telemetryUpdate(telemetry, "Green Input (Color sensor)", String.valueOf(AttachmentControl.color.green()));
            telemetryControl.telemetryUpdate(telemetry, "Blue Input (Color sensor)", String.valueOf(AttachmentControl.color.blue()));
            telemetryControl.motorTelemetryUpdate(telemetry, flpower, frpower, blpower, brpower);
            telemetryControl.update(telemetry);

        }

    }

}
