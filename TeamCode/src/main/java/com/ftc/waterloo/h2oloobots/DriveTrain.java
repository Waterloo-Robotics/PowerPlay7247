package com.ftc.waterloo.h2oloobots;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// configures all things drivetrain for teleOp opmodes, all of autonomous is performed through roadrunner (see encoder.odo.ftc.rr)
@Config
public class DriveTrain {

    public DcMotorEx fl, fr, bl, br;

    public DcMotorEx left, right;

    double COUNTS_PER_INCH;

    double COUNTS_PER_DEGREE;

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT
    }

    int fldirweight, frdirweight, bldirweight, brdirweight;

    public static double flweightco = 1.0;
    public static double frweightco = 1.0;
    public static double blweightco = 1.0;
    public static double brweightco = 1.0;
    double cpr = 537.7;
    int counter = 0;
    double flvelo, frvelo, blvelo, brvelo;
    double flveloavg, frveloavg, blveloavg, brveloavg;

    public DriveTrain(HardwareMap hardwareMap, TelemetryControl telemetryControl) {

        FourMotorInit(false, hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // initialises code for 2 motor drivebase
    public void TwoWheelInit(boolean RUN_USING_ENCODER, @NonNull HardwareMap hardwareMap) {

        left = (DcMotorEx) hardwareMap.dcMotor.get("left");
        right = (DcMotorEx) hardwareMap.dcMotor.get("right");

        if (RUN_USING_ENCODER) {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // simple teleOp for a 2 wheel drivebase, might need negatives and positives tuned based on drive base
    public void TwoWheelDriveTeleOp(double FBInput, double PivotInput, boolean RUN_USING_ENCODER) {

        right.setPower(FBInput - PivotInput);
        left.setPower(-FBInput - PivotInput);

    }

    // initialises any 4 motor drive base: tank, mecanum, or other
    public void FourMotorInit(boolean RUN_USING_ENCODER, @NonNull HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        fl = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        bl = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        br = (DcMotorEx) hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        // use any of the following if motors need reversed
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);

        if (RUN_USING_ENCODER) {
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Four Motor Init without encoder initialisation
    public void FourMotorInit(@NonNull HardwareMap hardwareMap) {

        fl = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        bl = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        br = (DcMotorEx) hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

    // non-mecanum 4 wheel drive teleOp
    public void FWDTeleOp(double FBInput, double PivotInput, boolean RUN_USING_ENCODER) {

        fr.setPower(-FBInput - PivotInput);
        fl.setPower(FBInput - PivotInput);
        br.setPower(-FBInput - PivotInput);
        bl.setPower(FBInput - PivotInput);

    }

    double speedMul = 0.75;

    // mecanum teleOp, with no speed adjusters on this end
    public void MecanumTeleOp(double FBInput, double LRInput, double PivotInput) {

        fl.setPower(speedMul * (-FBInput + LRInput + (PivotInput)));
        bl.setPower(speedMul * (-FBInput - LRInput + (PivotInput)));
        fr.setPower(speedMul * (FBInput + LRInput + (PivotInput)));
        br.setPower(speedMul * (FBInput - LRInput + (PivotInput)));

    }

    // mecanum TeleOp, with speed adjuster built in
    public void MecanumTeleOp(double FBInput, double LRInput, double PivotInput, double speed) {

        fl.setPower(speed * (-FBInput + LRInput + (PivotInput)));
        bl.setPower(speed * (-FBInput - LRInput + (PivotInput)));
        fr.setPower(speed * (FBInput + LRInput + (PivotInput)));
        br.setPower(speed * (FBInput - LRInput + (PivotInput)));

    }

    // Initialiser for Encoders, accounting Wheel Diameter, Gear Ratio, and Motor Counts per revolution to calculate counts per inch and counts per degree (rotation)
    public void EncoderAutoInit(double WHEEL_DIAMETER_MM, double GEAR_RATIO, double COUNTS_PER_REVOLUTION) {

        double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;

        COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415);

        // 50 here may need adjusted to how many inches are read to turn 90 degrees
        COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * 50) / 90;

    }

    // use in the case of not wanting to establish wheel diameter every time, just update the "100 / 25.4" to what is needed
    public void EncoderAutoInit(double GEAR_RATIO, double COUNTS_PER_REVOLUTION) {

        double WHEEL_DIAMETER_INCHES = 100 / 25.4;

        COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415);

        COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * 50) / 90;

    }

    // establish powers of all 4 motors, and how long to run for for timed autonomous commands
    public void timeAutoMecanumDrive(double FRPower, double FLPower, double BRPower, double BLPower, double SECONDS) {

        ElapsedTime time = new ElapsedTime();

        time.reset();

        while (time.seconds() < SECONDS) {

            fr.setPower(FRPower);
            fl.setPower(FLPower);
            br.setPower(BRPower);
            bl.setPower(BLPower);

        }

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        // uncomment this code if previous movements are bleeding into the next, causing unpredictable motion
//        ElapsedTime waitTimer = new ElapsedTime();
//        waitTimer.reset();
//        while (waitTimer.seconds() <= 0.125) {
//
//        }

    }

    public void EncoderAutoMecanumDrive(double INCHES_FB, double INCHES_LR, double DEGREES_TURN, double SPEED, int time) {

        ElapsedTime timer = new ElapsedTime();

        if (DEGREES_TURN > 180) {

            DEGREES_TURN -= 360;

        }

        int frTargetPosition = fr.getCurrentPosition() + (int) (COUNTS_PER_INCH * INCHES_FB) - (int) (COUNTS_PER_INCH * INCHES_LR) - (int) (COUNTS_PER_DEGREE * DEGREES_TURN);
        int brTargetPosition = br.getCurrentPosition() + (int) (COUNTS_PER_INCH * INCHES_FB) + (int) (COUNTS_PER_INCH * INCHES_LR) - (int) (COUNTS_PER_DEGREE * DEGREES_TURN);
        int flTargetPosition = fl.getCurrentPosition() - (int) (COUNTS_PER_INCH * INCHES_FB) - (int) (COUNTS_PER_INCH * INCHES_LR) - (int) (COUNTS_PER_DEGREE * DEGREES_TURN);
        int blTargetPosition = bl.getCurrentPosition() - (int) (COUNTS_PER_INCH * INCHES_FB) + (int) (COUNTS_PER_INCH * INCHES_LR) - (int) (COUNTS_PER_DEGREE * DEGREES_TURN);

        fr.setTargetPosition(frTargetPosition);
        br.setTargetPosition(brTargetPosition);
        fl.setTargetPosition(flTargetPosition);
        bl.setTargetPosition(blTargetPosition);

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();

        fr.setPower(SPEED);
        br.setPower(SPEED);
        fl.setPower(SPEED);
        bl.setPower(SPEED);

        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && timer.seconds() <= time) {

        }

        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // uncomment this code if previous movements are bleeding into the next, causing unpredictable motion
//        ElapsedTime waitTimer = new ElapsedTime();
//        waitTimer.reset();
//        while (waitTimer.seconds() <= 0.125) {
//
//        }

    }

    // honestly ignore this lol, this was to tune the motors for the weight so when the robot was supposed to move straight it would but there are flaws in it and I don't feel like fixing them right now it's 23:00
    public void weightConfig(Telemetry telemetry, TelemetryControl telemetryControl, double flpos, double frpos, double blpos, double brpos, double speed, @NonNull Direction direction, double DIST_REV) {

        flveloavg = 0;
        frveloavg = 0;
        blveloavg = 0;
        brveloavg = 0;

        flvelo = 0;
        frvelo = 0;
        blvelo = 0;
        brvelo = 0;

        switch (direction) {
            case BACKWARD:
                fldirweight = -1;
                frdirweight = 1;
                bldirweight = -1;
                brdirweight = 1;
                break;

            case FORWARD:
                fldirweight = 1;
                frdirweight = -1;
                bldirweight = 1;
                brdirweight = -1;
                break;

            case RIGHT:
                fldirweight = 1;
                frdirweight = 1;
                bldirweight = -1;
                brdirweight = -1;
                break;

            case LEFT:
                fldirweight = -1;
                frdirweight = -1;
                bldirweight = 1;
                brdirweight = 1;
                break;

        }

        fl.setTargetPosition((int) (fl.getCurrentPosition() + (fldirweight * cpr) * DIST_REV));
        fr.setTargetPosition((int) (fr.getCurrentPosition() + (frdirweight * cpr) * DIST_REV));
        bl.setTargetPosition((int) (bl.getCurrentPosition() + (bldirweight * cpr) * DIST_REV));
        br.setTargetPosition((int) (br.getCurrentPosition() + (brdirweight * cpr) * DIST_REV));

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (fl.getCurrentPosition() != fl.getTargetPosition() && fr.getCurrentPosition() != fr.getTargetPosition() && bl.getCurrentPosition() != bl.getTargetPosition() && br.getCurrentPosition() != br.getTargetPosition()){
            counter++;
            flpos = fl.getCurrentPosition();
            flvelo = fl.getVelocity(AngleUnit.DEGREES);
            flveloavg = (flveloavg + flvelo);
            fl.setPower(speed * flweightco * fldirweight);
            telemetryControl.telemetryUpdate("fl pos", String.valueOf(flpos));
            telemetryControl.telemetryUpdate("fl velo", String.valueOf(flvelo));
            telemetryControl.telemetryUpdate("flveloavg", String.valueOf(flveloavg));
            telemetryControl.telemetryUpdate("fl avg velo", String.valueOf(flveloavg / counter));
            frpos = fr.getCurrentPosition();
            frvelo = fr.getVelocity(AngleUnit.DEGREES);
            frveloavg = (frveloavg + frvelo);
            fr.setPower(speed * frweightco * frdirweight);
            telemetryControl.telemetryUpdate("fr pos", String.valueOf(frpos));
            telemetryControl.telemetryUpdate("fr velo", String.valueOf(frvelo));
            telemetryControl.telemetryUpdate("fr avg velo", String.valueOf(frveloavg / counter));
            blpos = bl.getCurrentPosition();
            blvelo = bl.getVelocity(AngleUnit.DEGREES);
            blveloavg = (blveloavg + blvelo);
            bl.setPower(speed * blweightco * bldirweight);
            telemetryControl.telemetryUpdate("bl pos", String.valueOf(blpos));
            telemetryControl.telemetryUpdate("bl velo", String.valueOf(blvelo));
            telemetryControl.telemetryUpdate("bl avg velo", String.valueOf(blveloavg / counter));
            brpos = br.getCurrentPosition();
            brvelo = br.getVelocity(AngleUnit.DEGREES);
            brveloavg = (brveloavg + brvelo);
            br.setPower(speed * brweightco * brdirweight);
            telemetryControl.telemetryUpdate("br pos", String.valueOf(brpos));
            telemetryControl.telemetryUpdate("br velo", String.valueOf(brvelo));
            telemetryControl.telemetryUpdate("br avg velo", String.valueOf(brveloavg / counter));
            telemetryControl.update();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        try {
            sleep(30000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
