package org.firstinspires.ftc.teamcode.Tests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

@Disabled
@Config
@TeleOp
public class WeightDistTest extends LinearOpMode {

//    public DcMotorEx fl, fr, bl, br;

    public static double speed = 0.5;

    int flpos, frpos, blpos, brpos;

    int motorpos;

    double flvelo, frvelo, blvelo, brvelo;

    double flveloavg, frveloavg, blveloavg, brveloavg;

    double cpr = 537.7;

    int counter = 1;

    public static double revs = 10.0;

    public static DriveTrain.Direction direction = DriveTrain.Direction.FORWARD;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode(){ // Code to try to speed up/down motors to accommodate for weight distribution

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl);

        waitForStart();

        while (opModeIsActive()) {

            driveTrain.weightConfig(telemetry, telemetryControl, flpos, frpos, blpos, brpos, speed, direction, revs);

        }

    }

}
