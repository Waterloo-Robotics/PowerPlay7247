package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

// All telemetry used by our code goes through here
public class TelemetryControl {

    // initialise variables required for FtcDashboard (http://192.168.43.1:8080/dash)
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

    Telemetry telemetryLocal;

    // configuring telemetry to a local value so we don't have to request it every time
    public TelemetryControl(Telemetry telemetry) {

        packet.addLine("Robot Initialised");
        telemetry.addLine("Robot Initialised");
        telemetryLocal = telemetry;

    }

    // typical telemetry.addData line with a label/title and a value
    public void addData(String caption, Object value) {

        telemetryLocal.addData(caption, value);
        packet.put(caption, value);

    }

    public void addLine(String line) {

        telemetryLocal.addLine(line);
        packet.clearLines();
        packet.addLine(line);

    }

    // motor direction obtainer for debugging purposes, positives and negatives will need flipped based on drive base
    public void motorTelemetryUpdate(double flpower, double frpower, double blpower, double brpower) {

        fldir = getDirection(flpower);
        frdir = getDirection(frpower);
        bldir = getDirection(blpower);
        brdir = getDirection(brpower);

        double frontMin = Math.min(fldir, frdir);
        double backMin = Math.min(bldir, brdir);

        String direction = "";
        double leftMax = Math.max(flpower, blpower);
        double rightMax = Math.max(frpower, brpower);
        packet.clearLines();
        if (fldir != 0 && frdir != 0 && bldir != 0 && brdir != 0) {
            if (fldir == -1 && bldir == -1 && frdir == 1 && brdir == 1)
                direction = "Moving Backward";
            if (fldir == 1 && bldir == 1 && frdir == -1 && brdir == -1)
                direction = "Moving Forward";
            if (fldir == 1 && bldir == -1 && frdir == 1 && brdir == -1)
                direction = "Strafing Right";
            if (fldir == -1 && bldir == 1 && frdir == -1 && brdir == 1)
                direction = "Strafing Left";
            if (fldir == 1 && bldir == 1 && frdir == 1 && brdir == 1)
                direction = "Turning Right";
            if (fldir == -1 && bldir == -1 && frdir == -1 && brdir == -1)
                direction = "Turning Left";
            if (frontMin == 0 && backMin == 0)
                direction = "Moving Diagonally";
            if ((frontMin == 0 && backMin != 0) || (backMin == 0 && frontMin != 0))
                direction = "Moving Strangely";
            telemetryLocal.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
            packet.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
        } else {

            telemetryLocal.addLine("Stopped");
            packet.addLine("Stopped");

        }

    }

    // starts camera stream, no clue why this is used instead of just the single line but whatever it appears to be used so I'm not gonna touch it
    public void startCameraStream(CameraStreamSource source, double maxFps) {

        dashboard.startCameraStream(source, maxFps);

    }

    // updates telemetry and dashboard to show new values
    public void update() {

        telemetryLocal.update();
        dashboard.sendTelemetryPacket(packet);

    }

    // gets which direction the motor is running (1 is clockwise and -1 is anticlockwise when the motor directions are all set to FORWARD
    public double getDirection(double motorPower) {

        if (motorPower != 0) {

            return (motorPower) / Math.abs(motorPower);

        } else {

            return 0;

        }

    }

}
