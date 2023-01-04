package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
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

@TeleOp
@Config
public class TeleOpComp extends LinearOpMode {
    double flpower, frpower, blpower, brpower;
    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

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

    BaRc.Labels label = BaRc.Labels.RED;

    private static final String VUFORIA_KEY =
            "ARZRc7b/////AAABmdYJzscFYUPwkOkgBgmpjAgFqkg6LxHmXPnonXKPGrqHxZBHhNRmsyDoNVz/o9XfL9Dc9224rfZHzQ5gvbFPQYchZztz+A+SeQt4Ql4xUxAiYviidC0RpPC9TRRBOaaGlNLAAupCmG8jtnsbnLlVFECI4DC3Heg20nvyWce+A4anUbW+kFkQu9uGyD1JzToWPN8MV7wsnsJ+1/oe6F7+g8C9dtMW3MgktiWWuty0g4RLNrc4qz/3htZe6w2efr1e34EZ+0JJc9la+bV08IMlSlEKGy2QOPBUIODemnFxyu+JLAkleQtV+Pke4qOf9hm8jTeqhyu5rL7B9QVZQ1x5p/u1vSsFJs2ihWJ9O1TvtPCs";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode() {

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

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.open, true, true);

        // Declaring variables for joystick controls (greyed out ones are no longer used)

        double shDir = 0; // shoulder direction - rotates up (positive numbers) or rotates down (negative numbers)

        double elDir = 0; // elbow direction- rotates down and extends out (positive numbers) or rotates up and folds in (negative numbers)

        double wristDir = 0; // wrist direction - rotate clockwise (positive numbers) or counterclockwise . View is from the front of the motors.

        boolean isAPushed = false; // automatic pickup position for arm and elbow

        boolean pickUp = false;

        boolean up = false;

        boolean score = false;

        boolean claw = false;

        boolean isBPushed = false; // toggle for opening/closing the claw. Holding the B button just makes the claw close.

        boolean yPressed = false;

        double speedMul = 0.55;

        boolean isBackPressed = false;

        boolean isBackPressedOnce = false;

        ElapsedTime backTimer = new ElapsedTime();

        boolean autoEnabled = true;

        boolean isStartPressed = false;

        boolean isStartPressedOnce = false;

        ElapsedTime startTimer = new ElapsedTime();

        boolean colorEnabled = true;

        boolean home = true;

        boolean isAPressed = false;

        boolean isAPressedOnce = false;

        ElapsedTime aTimer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetryControl.addData("# Objects Detected", String.valueOf(updatedRecognitions.size()));

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
//                    for (Recognition recognition : updatedRecognitions) {
//                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
//                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
//
//                        telemetry.addData(""," ");
//                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
//                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
//                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
//                    }
//                    telemetry.update();
                }
            }

            if (gamepad1.right_bumper) speedMul = 1; else speedMul = 0.55;

//            if (gamepad1.y) yPressed = true;
//
//            if (gamepad2.dpad_up) shDir = 1; else if (gamepad2.dpad_down) shDir = -1; else shDir = 0;
//
//            if (gamepad2.y) elDir = -1; else if (gamepad2.a) elDir = 1; else elDir = 0;
//
//            if (gamepad2.right_bumper/* && AttachmentControl.wrist.getCurrentPosition() > 0*/) wristDir = -1; else if (gamepad2.left_bumper/* && AttachmentControl.wrist.getCurrentPosition() < 200*/) wristDir = 1; else wristDir = 0;
//
//            if (gamepad2.b && !isBPushed) {
//
//                claw = !claw;
//
//                isBPushed = true;
//            } else if (!gamepad2.b) {
//
//                isBPushed = false;
//
//            }
//
//
//            // TODO Fix this
//            if (gamepad2.a) {
//
//                pickUp = true;
//                score = false;
//                up = true;
//
//            } else if (gamepad2.y) {
//
//                pickUp = false;
//                score = true;
//                up = true;
//
//            } else {
//
//                pickUp = false;
//
//            }
            score = false;
//
//
//            attachmentControl.armAuto(pickUp, score, claw);
//            if (gamepad2.right_bumper) { attachmentControl.armToCatch(); }
//
//            if (gamepad2.left_bumper) { attachmentControl.armToScore(); }

            if (gamepad2.dpad_up) shDir = 1; else if (gamepad2.dpad_down) shDir = -1; else shDir = 0;

            if (gamepad2.y) elDir = -1; else if (gamepad2.a) elDir = 1; else elDir = 0;

            // Changing Wrist Direction
            if (gamepad2.right_trigger > 0.1) wristDir = -gamepad2.right_trigger; else if (gamepad2.left_trigger > 0.1) wristDir = gamepad2.left_trigger; else wristDir = 0;
            if (gamepad2.b && !isBPushed) {

                if (AttachmentControl.clawAuto) claw = false; else claw = !claw;

                isBPushed = true;

            } else if (!gamepad2.b) {

                isBPushed = false;

            }

            if (gamepad2.back && !isBackPressed) {

                if (isBackPressedOnce && backTimer.seconds() < 0.67) {

                    autoEnabled = !autoEnabled;

                    gamepad2.rumbleBlips(3);

                    isBackPressedOnce = false;

                } else {

                    if (autoEnabled) {

                        gamepad2.rumbleBlips(1);

                    } else {

                        gamepad2.rumbleBlips(2);

                    }

                    backTimer.reset();

                    isBackPressedOnce = true;

                }

                isBackPressed = true;

            } else if (!gamepad2.back) {

                isBackPressed = false;

            }

            if (gamepad2.start && !isStartPressed) {

                if (isStartPressedOnce && startTimer.seconds() < 0.67) {

                    colorEnabled = !colorEnabled;

                    gamepad2.rumbleBlips(4);

                    isStartPressedOnce = false;

                } else {

                    if (colorEnabled) {

                        gamepad2.rumbleBlips(1);

                    } else {

                        gamepad2.rumbleBlips(2);

                    }

                    startTimer.reset();

                    isStartPressedOnce = true;

                }

                isStartPressed = true;

            } else if (!gamepad2.start) {

                isStartPressed = false;

            }

            if (gamepad1.a && !isAPressed) {

                if (isAPressedOnce && aTimer.seconds() < 0.67) {

                    home = true;

                    isAPressedOnce = false;

                } else {

                    aTimer.reset();

                    isAPressedOnce = true;

                }

                isAPressed = true;

            } else if (!gamepad1.a) {

                isAPressed = false;

                home = false;

            }

//            attachmentControl.touchSensor();

            attachmentControl.armCompWithAutomation(home, autoEnabled, gamepad2.dpad_down || gamepad2.a, gamepad2.right_bumper, gamepad2.left_bumper, claw, gamepad2.b, -gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, gamepad2.dpad_left, gamepad2.dpad_right);

            pickUp = false;
            score = false;

//            attachmentControl.armManualComp(-gamepad2.left_stick_y, gamepad2.right_stick_y, wristDir, claw, telemetryControl);

            flpower = driveTrain.fl.getPower();
            frpower = driveTrain.fr.getPower();
            blpower = driveTrain.bl.getPower();
            brpower = driveTrain.br.getPower();

            driveTrain.MecanumTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speedMul);
            telemetryControl.update();

        }

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
