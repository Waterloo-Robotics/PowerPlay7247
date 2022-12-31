package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.regex.Matcher;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;
import encoder.odo.ftc.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class SidewaysAutoTest extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();



    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/SignalSleeve.tflite";

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final String[] LABELS = {
            "Red",
            "Green",
            "Blue"
    };

    public enum Labels {
        RED,
        GREEN,
        BLUE
    }

    PatrickAutoAwesomeness.Labels label = PatrickAutoAwesomeness.Labels.RED;

    private static final String VUFORIA_KEY =
            "ARZRc7b/////AAABmdYJzscFYUPwkOkgBgmpjAgFqkg6LxHmXPnonXKPGrqHxZBHhNRmsyDoNVz/o9XfL9Dc9224rfZHzQ5gvbFPQYchZztz+A+SeQt4Ql4xUxAiYviidC0RpPC9TRRBOaaGlNLAAupCmG8jtnsbnLlVFECI4DC3Heg20nvyWce+A4anUbW+kFkQu9uGyD1JzToWPN8MV7wsnsJ+1/oe6F7+g8C9dtMW3MgktiWWuty0g4RLNrc4qz/3htZe6w2efr1e34EZ+0JJc9la+bV08IMlSlEKGy2QOPBUIODemnFxyu+JLAkleQtV+Pke4qOf9hm8jTeqhyu5rL7B9QVZQ1x5p/u1vSsFJs2ihWJ9O1TvtPCs";

//    private VuforiaLocalizer vuforia;
//
//    private TFObjectDetector tfod;

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false, true);

//        dashboard.startCameraStream(tfod, 24);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(3,0,0);



        drive.setPoseEstimate(startPose);

        TrajectorySequence drive_to_stack = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-5, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(5, 80, Math.toRadians(-5)))
                .lineToLinearHeading(new Pose2d(13, 80, Math.toRadians(-5)))
//                .turn(Math.toRadians(-90))
//                .forward(4)
//                .strafeLeft(7)
                .build();

        waitForStart();

        timer.reset();

//        while (timer.seconds() < 2) {
//
//            telemetryControl.startCameraStream(tfod, 24);
//
//            if (tfod != null) {
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetryControl.telemetryUpdate("# Objects Detected", String.valueOf(updatedRecognitions.size()));
//                    if (updatedRecognitions.size() == 1) {
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals("Green")) {
//                                label = PatrickAutoAwesomeness.Labels.GREEN;
//                            } else if (recognition.getLabel().equals("Blue")) {
//                                label = PatrickAutoAwesomeness.Labels.BLUE;
//                            } else {
//                                label = PatrickAutoAwesomeness.Labels.RED;
//                            }
//                        }
//                    }
//                    telemetryControl.telemetryUpdate("Identification", String.valueOf(label));
//                    telemetryControl.update();
//                }
//            }
//        }

        drive.followTrajectorySequence(drive_to_stack);
        drive.setMotorPowers(0,0,0,0); // about 10
        attachmentControl.setArmPositions(3903, -2584, 0, true); //To Score
        attachmentControl.setArmPositions(3903, -2584, -726, true); //To Score
        this.servo(attachmentControl, 1); //Open
//        attachmentControl.setArmPositions(0, 0, 0, true);
        attachmentControl.setArmPositions(185, -3424, -21, true); //To Pickup
        this.servo(attachmentControl,0);
        attachmentControl.setArmPositions(3903, -2584, 0, true); //To Score
        attachmentControl.setArmPositions(3903, -2584, -726, true); //To Score
        this.servo(attachmentControl, 1);
        attachmentControl.setArmPositions(198, -3686, -21, true); //To Pickup
        this.servo(attachmentControl,0);
        attachmentControl.setArmPositions(3903, -2584, 0, true); //To Score
        attachmentControl.setArmPositions(3903, -2584, -726, true); //To Score
        this.servo(attachmentControl, 1);

    }

    public void score(AttachmentControl attachmentControl) { // rudimentary scoring position (medium junction)

        attachmentControl.setArmPositions(3899, -2404, -726, true);
        attachmentControl.openClaw();

    }

    public void pickup(AttachmentControl attachmentControl) { // down position to prevent hitting the wall

        attachmentControl.setArmPositions(0, -3348, -11, true);
        attachmentControl.closeClaw();

    }

    public void servo(AttachmentControl attachmentControl, double position) { // servo setposition function (yes I know there's a function for closing and opening the servo I forgot and am too lazy to fix it

        AttachmentControl.claw.setPosition(position);

        timer.reset();
        while (timer.seconds() < 0.375) {}

    }

}
