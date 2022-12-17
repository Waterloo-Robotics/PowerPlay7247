package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;
import encoder.odo.ftc.rr.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Same as BaRC but good!!")
public class PatrickAutoAwesomeness extends LinearOpMode {

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

    Labels label = Labels.RED;

    private static final String VUFORIA_KEY =
            "ARZRc7b/////AAABmdYJzscFYUPwkOkgBgmpjAgFqkg6LxHmXPnonXKPGrqHxZBHhNRmsyDoNVz/o9XfL9Dc9224rfZHzQ5gvbFPQYchZztz+A+SeQt4Ql4xUxAiYviidC0RpPC9TRRBOaaGlNLAAupCmG8jtnsbnLlVFECI4DC3Heg20nvyWce+A4anUbW+kFkQu9uGyD1JzToWPN8MV7wsnsJ+1/oe6F7+g8C9dtMW3MgktiWWuty0g4RLNrc4qz/3htZe6w2efr1e34EZ+0JJc9la+bV08IMlSlEKGy2QOPBUIODemnFxyu+JLAkleQtV+Pke4qOf9hm8jTeqhyu5rL7B9QVZQ1x5p/u1vSsFJs2ihWJ9O1TvtPCs";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.25, 16.0/9.0);
        }

        dashboard.startCameraStream(tfod, 24);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        timer.reset();

        while (timer.seconds() < 2) {

            telemetryControl.startCameraStream(tfod, 24);

            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetryControl.telemetryUpdate("# Objects Detected", String.valueOf(updatedRecognitions.size()));

                    if (updatedRecognitions.size() == 1) {
                        // setting the different Y-axis coordinates for the different detected object detected on the signal sleeve.
                        // Y-axis is left/right of the robot. Left is positive value.
                        // The value sets the robot strafing position. The coordinate value is in inch (accuracy questionable).

                        for (Recognition recognition : updatedRecognitions) {

                            if (recognition.getLabel() == "Green") {

                                label = Labels.GREEN;

                                //parky = 4; //Set y position to 4 inches to the left of the robot from starting position.

                            } else if (recognition.getLabel() == "Blue") {

                                label = Labels.BLUE;

                                //parky = 27; //Set y position to 27 inches to the left of the robot from starting position.

                                // currently not used parkh = 100;

                            } else {

                                label = Labels.RED;

                                //parky = -18; //Set y position to 18 inches to the right of the robot from starting position.

                            }

                        }

                    }
                    telemetryControl.telemetryUpdate("Identification", String.valueOf(label));
                    telemetryControl.update();

                }
            }
        }

        Pose2d startPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //.splineTo(new Vector2d(0,6),0)
                .strafeLeft(4)
                .forward(50)
                .turn(Math.toRadians(90))
                .forward(5)
                .strafeLeft(3)
                .build();


        //Calculate trajectory for scoring starting cone
//        Trajectory shift_left = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(0,5,0))
//                .build();
//
//        Trajectory approach_center = drive.trajectoryBuilder(shift_left.end())
//                .lineToLinearHeading(new Pose2d(50,0,0))
//                        .build();
//
//        Trajectory approach_tape = drive.trajectoryBuilder(approach_center.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
//                .lineToLinearHeading(new Pose2d(1,0,0))
//                        .build();
//
//        drive.followTrajectory(shift_left);
//        drive.followTrajectory(approach_center);
//        drive.turn(Math.toRadians(90));
//        drive.followTrajectory(approach_tape);
        drive.followTrajectorySequence(trajSeq);
        drive.setMotorPowers(0,0,0,0);
        attachmentControl.setArmPositions(4107,-1082,-732,true);
        sleep(1000);


        sleep(3000);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 100;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}
