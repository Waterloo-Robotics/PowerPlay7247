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

@Config
@Autonomous(name = "Red Corner")
public class BaRc extends LinearOpMode {

    double turn1 = 115;
    public static double strafey = -20;

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

    double parky = -20;

    double parkh = 95;

//    private static final String[] LABELS = {
//            "Blue Cone"
//    };

    private static final String VUFORIA_KEY =
            "ARZRc7b/////AAABmdYJzscFYUPwkOkgBgmpjAgFqkg6LxHmXPnonXKPGrqHxZBHhNRmsyDoNVz/o9XfL9Dc9224rfZHzQ5gvbFPQYchZztz+A+SeQt4Ql4xUxAiYviidC0RpPC9TRRBOaaGlNLAAupCmG8jtnsbnLlVFECI4DC3Heg20nvyWce+A4anUbW+kFkQu9uGyD1JzToWPN8MV7wsnsJ+1/oe6F7+g8C9dtMW3MgktiWWuty0g4RLNrc4qz/3htZe6w2efr1e34EZ+0JJc9la+bV08IMlSlEKGy2QOPBUIODemnFxyu+JLAkleQtV+Pke4qOf9hm8jTeqhyu5rL7B9QVZQ1x5p/u1vSsFJs2ihWJ9O1TvtPCs";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.25, 16.0/9.0);
        }

        dashboard.startCameraStream(tfod, 24);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        timer.reset();

        while (timer.seconds() < 2) {

            dashboard.startCameraStream(tfod, 24);

            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    for (Recognition recognition : updatedRecognitions) {

                        if (updatedRecognitions.size() == 1) {

                            if (recognition.getLabel() == "Green") {

                                label = Labels.GREEN;

                                parky = 4;

                            } else if (recognition.getLabel() == "Blue") {

                                label = Labels.BLUE;

                                parky = 27;

                                parkh = 100;

                            } else {

                                label = Labels.RED;

                                parky = -20;

                            }

                        } else if (updatedRecognitions.size() == 2) {

                            if (recognition.getLabel() == "Blue") {

                                label = Labels.BLUE;

                                parky = 27;

                            } else if (recognition.getLabel() == "Red"){

                                label = Labels.RED;

                                parky = -20;

                            } else {

                                label = Labels.GREEN;

                                parky = 10;

                            }

                        }

                    }
                    telemetryControl.telemetryUpdate("Identification", String.valueOf(label));
                    telemetryControl.update();

                }
            }
        }

        Trajectory strafe1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, parky, 0))
                .build();

        Trajectory moveForward1 = drive.trajectoryBuilder(strafe1.end())
                .lineToLinearHeading(new Pose2d(30, parky, 0))
                .build();

//        Trajectory moveToBlack = drive.trajectoryBuilder(new Pose2d(48, strafey, Math.toRadians(turn1)))
//                .lineToLinearHeading(new Pose2d(52, 16, Math.toRadians(turn1)))
//                .build();

//        Trajectory park = drive.trajectoryBuilder(new Pose2d(22, strafey, Math.toRadians(parkh)))
//                        .lineToLinearHeading(new Pose2d(23, parky, Math.toRadians(parkh)))
//                        .build();

        drive.followTrajectory(strafe1);

        drive.followTrajectory(moveForward1);

//        drive.turn(Math.toRadians(parkh));

//        AttachmentControl.shoulder.setTargetPosition(3931);
//        AttachmentControl.elbow.setTargetPosition(1945);
//        AttachmentControl.wrist.setTargetPosition(-116);

//        AttachmentControl.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        AttachmentControl.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        AttachmentControl.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (AttachmentControl.shoulder.getCurrentPosition() != AttachmentControl.shoulder.getTargetPosition() && AttachmentControl.elbow.getCurrentPosition() != AttachmentControl.elbow.getTargetPosition() /*&& AttachmentControl.wrist.getCurrentPosition() != AttachmentControl.wrist.getTargetPosition()*/) {
//
//            AttachmentControl.shoulder.setPower(1);
//            AttachmentControl.elbow.setPower(1);
//            AttachmentControl.wrist.setPower(1);
//
//        }
//
//        AttachmentControl.shoulder.setPower(0);
//        AttachmentControl.elbow.setPower(0);
//        AttachmentControl.wrist.setPower(0);
//
//        AttachmentControl.claw.setPosition(0);
//
//        sleep(1000);
//
//        AttachmentControl.shoulder.setTargetPosition(0);
//        AttachmentControl.elbow.setTargetPosition(0);
//        AttachmentControl.wrist.setTargetPosition(0);
//
//        AttachmentControl.shoulder.setPower(1);
//        AttachmentControl.elbow.setPower(1);
//        AttachmentControl.wrist.setPower(1);

//        drive.followTrajectory(park);

//        while (AttachmentControl.shoulder.getCurrentPosition() != AttachmentControl.shoulder.getTargetPosition() && AttachmentControl.elbow.getCurrentPosition() != AttachmentControl.elbow.getTargetPosition() /*&& AttachmentControl.wrist.getCurrentPosition() != AttachmentControl.wrist.getTargetPosition()*/) {
//
//            AttachmentControl.shoulder.setPower(1);
//            AttachmentControl.elbow.setPower(1);
//            AttachmentControl.wrist.setPower(1);
//
//        }
//
//        AttachmentControl.shoulder.setPower(0);
//        AttachmentControl.elbow.setPower(0);
//        AttachmentControl.wrist.setPower(0);

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
