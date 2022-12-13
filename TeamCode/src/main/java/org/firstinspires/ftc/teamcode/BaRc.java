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
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false);

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

                                parky = 4; //Set y position to 4 inches to the left of the robot from starting position.

                            } else if (recognition.getLabel() == "Blue") {

                                label = Labels.BLUE;

                                parky = 27; //Set y position to 27 inches to the left of the robot from starting position.

                                // currently not used parkh = 100;

                            } else {

                                label = Labels.RED;

                                parky = -18; //Set y position to 18 inches to the right of the robot from starting position.

                            }

                        }

                    }
                    telemetryControl.telemetryUpdate("Identification", String.valueOf(label));
                    telemetryControl.update();

                }
            }
        }
        // Set strafe robot trajectory based on the Y-axis coordinate set based on the object recognized by the camera
        Trajectory strafe1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, parky, 0))
                .build();
        // Set forward robot trajectory to go to the center of the park zone
        Trajectory moveForward1 = drive.trajectoryBuilder(strafe1.end())
                .lineToLinearHeading(new Pose2d(30, parky, 0))
                .build();

        // Perform the strafing motion
        drive.followTrajectory(strafe1);
        // Perform the forward motion
        drive.followTrajectory(moveForward1);

        drive.setMotorPowers(0,0,0,0);
        sleep(3000);

        //attachmentControl.setArmPositions(4176, 3480, 703, true); //1800, 1254, 624
//        attachmentControl.setArmPositions(1800,1254,624, true);

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
