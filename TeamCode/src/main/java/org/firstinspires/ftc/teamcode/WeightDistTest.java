package org.firstinspires.ftc.teamcode;

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

    DriveTrain driveTrain = new DriveTrain();
    TelemetryControl telemetryControl = new TelemetryControl();

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
    public void runOpMode(){

        driveTrain.FourMotorInit(false, hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryControl.telemetryInit(telemetry);
//        fl = (DcMotorEx) hardwareMap.dcMotor.get("fl");
//        fr = (DcMotorEx) hardwareMap.dcMotor.get("fr");
//        bl = (DcMotorEx) hardwareMap.dcMotor.get("bl");
//        br = (DcMotorEx) hardwareMap.dcMotor.get("br");
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setTargetPosition((int) (cpr * 10));
//        fr.setTargetPosition((int) (-cpr * 10));
//        bl.setTargetPosition((int) (cpr * 10));
//        br.setTargetPosition((int) (-cpr * 10));
//
//        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            driveTrain.weightConfig(telemetry, telemetryControl, flpos, frpos, blpos, brpos, speed, direction, revs);
//            while (fl.getCurrentPosition() != fl.getTargetPosition() || fr.getCurrentPosition() != fr.getTargetPosition() || bl.getCurrentPosition() != bl.getTargetPosition() || br.getCurrentPosition() != br.getTargetPosition()){
//                if (counter >= 5377) counter = 5377;
//                flpos = fl.getCurrentPosition();
//                flvelo = fl.getVelocity() / cpr * 60;
//                flveloavg = (flveloavg + flvelo);
//                fl.setPower(speed);
//                telemetryControl.telemetryUpdate(telemetry, "fl pos", String.valueOf(flpos));
//                telemetryControl.telemetryUpdate(telemetry, "fl velo", String.valueOf(flvelo));
//                telemetryControl.telemetryUpdate(telemetry, "fl avg velo", String.valueOf(flveloavg / 5377));
//                frpos = fr.getCurrentPosition();
//                frvelo = fr.getVelocity() / cpr * 60;
//                frveloavg = (frveloavg + frvelo) / counter;
//                fr.setPower(-speed);
//                telemetryControl.telemetryUpdate(telemetry, "fr pos", String.valueOf(frpos));
//                telemetryControl.telemetryUpdate(telemetry, "fr velo", String.valueOf(frvelo));
//                telemetryControl.telemetryUpdate(telemetry, "fr avg velo", String.valueOf(frveloavg / 5377));
//                blpos = bl.getCurrentPosition();
//                blvelo = bl.getVelocity() / cpr * 60;
//                blveloavg = (blveloavg + blvelo);
//                bl.setPower(speed);
//                telemetryControl.telemetryUpdate(telemetry, "bl pos", String.valueOf(blpos));
//                telemetryControl.telemetryUpdate(telemetry, "bl velo", String.valueOf(blvelo));
//                telemetryControl.telemetryUpdate(telemetry, "bl avg velo", String.valueOf(blveloavg / 5377));
//                brpos = br.getCurrentPosition();
//                brvelo = br.getVelocity() / cpr * 60;
//                brveloavg = (brveloavg + brvelo);
//                br.setPower(-speed);
//                telemetryControl.telemetryUpdate(telemetry, "br pos", String.valueOf(brpos));
//                telemetryControl.telemetryUpdate(telemetry, "br velo", String.valueOf(brvelo));
//                telemetryControl.telemetryUpdate(telemetry, "br avg velo", String.valueOf(brveloavg / 5377));
//                telemetryControl.update(telemetry);
//                counter++;
//            }
//
//            fl.setPower(0);
//            fr.setPower(0);
//            bl.setPower(0);
//            br.setPower(0);

        }

    }

}
