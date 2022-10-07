package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleOpComp extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain();
    TelemetryControl telemetryControl = new TelemetryControl();
    double flpower, frpower, blpower, brpower;

    public void runOpMode() {

        driveTrain.FourMotorInit(false, hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        telemetryControl.telemetryInit(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetryControl.motorTelemetryUpdate(telemetry, flpower, frpower, blpower, brpower);
            telemetryControl.update(telemetry);

        }

    }

}
