package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp(name = "AprilTags")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
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

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
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

        dashboard.startCameraStream(camera, 60);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            dashboard.startCameraStream(camera, 60);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetryControl.addLine("Tag snapshot:");
            tagToTelemetry(tagOfInterest, telemetryControl);
            telemetryControl.update();
        }
        else
        {
            telemetryControl.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetryControl.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            //Park Left
        }else if(tagOfInterest.id == MIDDLE){
            //Park Middle
        }else{
            //Park Right
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection, TelemetryControl telemetryControl)
    {
        telemetryControl.addLine("Detected tag ID=" + detection.id);
    }
}
