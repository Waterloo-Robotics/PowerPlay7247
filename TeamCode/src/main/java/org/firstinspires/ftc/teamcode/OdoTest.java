package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class OdoTest extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;

    public static double P_gen = 0.03;
    public static double P_differential = 0.1;


    public static double MAX_POWER = 0.85;
    public static double MIN_POWER = 0.12;

    public static double DISTANCE = 53;

    public double INCHES_PER_COUNT = ((35.0 / 25.4) * Math.PI) / 8192;

    HardwareMap hardwareMapLocal;
    TelemetryControl telemetryControlLocal;

    ElapsedTime time = new ElapsedTime();

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);

        initialise(hardwareMap, true, telemetryControl);

        waitForStart();

        turn(DISTANCE);

    }

    public void initialise(HardwareMap hardwareMap, boolean RUN_USING_ENCODER, TelemetryControl telemetryControl) {

        fl = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        bl = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        br = (DcMotorEx) hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // use any of the following if motors need reversed
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);

        if (RUN_USING_ENCODER) {
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        hardwareMapLocal = hardwareMap;
        telemetryControlLocal = telemetryControl;

    }

    public void zeroEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    public void forward(double INCHES) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double differentialError = 0;

        double rightTravelled, leftTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        while (!destinationReached && !isStopRequested()) {

            rightTravelled = fr.getCurrentPosition() * INCHES_PER_COUNT;
            leftTravelled = -fl.getCurrentPosition() * INCHES_PER_COUNT;

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            differentialError = rightTravelled - leftTravelled;

            genPower = (genPower + (error * P_gen)) / 2.0;

            if (differentialError > 0) { //Turning left, right faster
                rightOffset = (rightOffset + ( (rightTravelled - leftTravelled) * P_differential) )/ 2.0;
                leftOffset = 0;
            } else {
                rightOffset = 0;
                leftOffset = (leftOffset + ( (leftTravelled - rightTravelled) * P_differential) ) / 2.0;
            }

            telemetryControlLocal.addData("Right", String.valueOf(rightTravelled));
            telemetryControlLocal.addData("Left", String.valueOf(leftTravelled));
            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset;
            leftPower = genPower + leftOffset;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER && rightPower > 0) {
                rightPower = MIN_POWER;
            } else if (rightPower > -MIN_POWER && rightPower < 0) {
                rightPower = -MIN_POWER;
            }

            if (leftPower < MIN_POWER && leftPower > 0) {
                leftPower = MIN_POWER;
            } else if (leftPower > -MIN_POWER && leftPower < 0) {
                leftPower = -MIN_POWER;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < 7);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void turn(double INCHES) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double differentialError = 0;

        double rightTravelled, leftTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        while (!destinationReached && !isStopRequested()) {

            rightTravelled = fr.getCurrentPosition() * INCHES_PER_COUNT;
            leftTravelled = fl.getCurrentPosition() * INCHES_PER_COUNT;

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            differentialError = rightTravelled + leftTravelled;

            genPower = (genPower + (error * P_gen)) / 2.0;

            if (differentialError > 0) { //Turning left, right faster
                rightOffset = (rightOffset + ( (leftTravelled - rightTravelled) * P_differential) )/ 2.0;
                leftOffset = 0;
            } else {
                rightOffset = 0;
                leftOffset = (leftOffset - ( (rightTravelled - leftTravelled) * P_differential) ) / 2.0;
            }

            telemetryControlLocal.addData("Right", String.valueOf(rightTravelled));
            telemetryControlLocal.addData("Left", String.valueOf(leftTravelled));
            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower - rightOffset;
            leftPower = -genPower + leftOffset;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER && rightPower > 0) {
                rightPower = MIN_POWER;
            } else if (rightPower > -MIN_POWER && rightPower < 0) {
                rightPower = -MIN_POWER;
            }

            if (leftPower < MIN_POWER && leftPower > 0) {
                leftPower = MIN_POWER;
            } else if (leftPower > -MIN_POWER && leftPower < 0) {
                leftPower = -MIN_POWER;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < 7);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

}
