package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {

    public DcMotor fl, fr, bl, br;

    public DcMotor left, right;

    double COUNTS_PER_INCH;

    double COUNTS_PER_DEGREE;

    public void TwoWheelInit(boolean RUN_USING_ENCODER, HardwareMap hardwareMap) {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        if (RUN_USING_ENCODER) {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void TwoWheelDriveTeleOp(double FBInput, double PivotInput, boolean RUN_USING_ENCODER) {

        right.setPower(FBInput - PivotInput);
        left.setPower(-FBInput - PivotInput);

    }

    public void FourMotorInit(boolean RUN_USING_ENCODER, HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

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

    public void FourMotorInit(HardwareMap hardwareMap) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

    public void FWDTeleOp(double FBInput, double PivotInput, boolean RUN_USING_ENCODER) {

        fr.setPower(-FBInput - PivotInput);
        fl.setPower(FBInput - PivotInput);
        br.setPower(-FBInput - PivotInput);
        bl.setPower(FBInput - PivotInput);

    }

    double speedMul = 1;

    public void MecanumTeleOp(double FBInput, double LRInput, double PivotInput, AttachmentControl attachmentControl) {

        fr.setPower(speedMul * (-FBInput - LRInput - (PivotInput)));
        br.setPower(speedMul * (-FBInput + LRInput - (PivotInput)));
        fl.setPower(speedMul * (FBInput - LRInput - (PivotInput)));
        bl.setPower(speedMul * (FBInput + LRInput - (PivotInput)));

    }

    public void EncoderAutoInit(double WHEEL_DIAMETER_MM, double GEAR_RATIO, double COUNTS_PER_REVOLUTION) {

        double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;

        COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415);

        COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * 50) / 90;

    }

    public void EncoderAutoInit(double GEAR_RATIO, double COUNTS_PER_REVOLUTION) {

        double WHEEL_DIAMETER_INCHES = 96 / 25.4;

        COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415);

        COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * 50) / 90;

    }

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

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() <= 0.125) {

        }

    }

}
