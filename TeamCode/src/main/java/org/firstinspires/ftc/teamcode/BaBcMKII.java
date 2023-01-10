package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.OdometryControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue Corner MK II")
public class BaBcMKII extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int detectionNumber = 0;

    AprilTagDetection tagOfInterest = null;

    ElapsedTime servoTimer = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        OdometryControl odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, AttachmentControl.ServoPosition.closed, false, true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        while (!isStarted() && !isStopRequested()) {

            telemetryControl.startCameraStream(camera, 60);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        detectionNumber = tag.id;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetryControl.addLine("Tag of interest is in sight! Location data:");
                    tagToTelemetry(tagOfInterest, telemetryControl);
                }
                else
                {
                    telemetryControl.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetryControl.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetryControl.addLine("But we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest, telemetryControl);
                    }
                }

            }
            else
            {
                telemetryControl.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetryControl.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetryControl.addLine("But we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest, telemetryControl);
                }

            }

            telemetryControl.update();
            sleep(20);
        }

//        odometryControl.back(2.5, 0.0625);
        odometryControl.strafeLeft(2.5, 0.0625);
        odometryControl.forward(50.5, 0.0625);
        odometryControl.turn(7.5, 0.0625);
        odometryControl.forward(6.5, 0.0625);
        odometryControl.strafeLeft(0.5, 0.0625);

        attachmentControl.setArmPositions(4000, -2199, 0, true); //To Score
        attachmentControl.setArmPositions(4000, -2199, -750, true); //To Score
        this.servo(1); //Open

        attachmentControl.setArmPositions(0, -2252, -15, false);
        timer.reset();
        while (timer.seconds() < 1.5);
        attachmentControl.setArmPositions(0, -3360, -15, true);
        this.servo(0); //Closed
//
        attachmentControl.setArmPositions(0, -2252, -15, false);
        timer.reset();
        while (timer.seconds() < 0.5);
        attachmentControl.setArmPositions(4000, -2199, 0, true); //To Score
        attachmentControl.setArmPositions(4000, -2199, -750, true); //To Score
        this.servo(1); //Open
//
        attachmentControl.setArmPositions(0, -2252, -15, false);
        timer.reset();
        while (timer.seconds() < 1.5);
        attachmentControl.setArmPositions(0, -3534, -6, true);
        this.servo(0); //Closed
//
        attachmentControl.setArmPositions(0, -2252, -6, false);
        timer.reset();
        while (timer.seconds() < 0.5);
        attachmentControl.setArmPositions(4000, -2199, 0, true); //To Score
        attachmentControl.setArmPositions(4000, -2199, -750, true); //To Score
        this.servo(1); //Open

        attachmentControl.setArmPositions(0, 0, 0, false);

        switch (detectionNumber) {

            case 3:

                attachmentControl.setArmPositions(0, 0, 0, true);
                odometryControl.forward(12, 0.0625);

                break;

            case 2:

                odometryControl.back(8, 0.0625);

                break;

            case 1:

                odometryControl.back(34, 0.0625);

                break;

        }

        attachmentControl.setArmPositions(0, 0, 0, true);


    }

    public void servo(double position) { // servo setposition function (yes I know there's a function for closing and opening the servo I forgot and am too lazy to fix it

        AttachmentControl.claw.setPosition(position);

        servoTimer.reset();
        while (servoTimer.seconds() < 0.375) {}

    }

    public void tagToTelemetry(AprilTagDetection detection, TelemetryControl telemetryControl) {

        telemetryControl.addLine("Detected tag ID=" + detection.id);

    }

}
