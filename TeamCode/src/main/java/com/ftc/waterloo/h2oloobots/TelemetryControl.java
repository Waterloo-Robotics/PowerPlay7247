package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

public class TelemetryControl {

    AttachmentControl attachmentControl = new AttachmentControl();
    DriveTrain driveTrain = new DriveTrain();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

//    Tele1 tele1 = new Tele1();

    public void telemetryInit(Telemetry telemetry) {

        packet.addLine("Robot Initialised");
        telemetry.addLine("Robot Initialised");

    }

    public void telemetryUpdate(Telemetry telemetry, String caption, String value) {

        telemetry.addData(caption, value);
        packet.put(caption, value);

    }

    public void motorTelemetryUpdate(Telemetry telemetry, double flpower, double frpower, double blpower, double brpower) {

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
                direction = "Moving Forward";
            if (fldir == 1 && bldir == 1 && frdir == -1 && brdir == -1)
                direction = "Moving Backward";
            if (fldir == 1 && bldir == -1 && frdir == 1 && brdir == -1)
                direction = "Strafing Left";
            if (fldir == -1 && bldir == 1 && frdir == -1 && brdir == 1)
                direction = "Strafing Right";
            if (fldir == 1 && bldir == 1 && frdir == 1 && brdir == 1)
                direction = "Turning Left";
            if (fldir == -1 && bldir == -1 && frdir == -1 && brdir == -1)
                direction = "Turning Right";
            if (frontMin == 0 && backMin == 0)
                direction = "Moving Diagonally";
            if ((frontMin == 0 && backMin != 0) || (backMin == 0 && frontMin != 0))
                direction = "Moving Strangely";
            telemetry.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
            packet.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
        } else {

            telemetry.addLine("Stopped");
            packet.addLine("Stopped");

        }

    }

    public void update(Telemetry telemetry) {

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

    }

    public double getDirection(double motorPower) {

        if (motorPower != 0) {

            return (motorPower) / Math.abs(motorPower);

        } else {

            return 0;

        }

    }

}
