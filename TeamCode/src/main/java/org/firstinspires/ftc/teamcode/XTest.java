package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;

@Autonomous
public class XTest extends LinearOpMode {
    public static double DISTANCE = 95; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(45)))
//                .build();

//        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
//                .back(DISTANCE)
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(45));

//        drive.followTrajectory(trajectory1);
//        drive.followTrajectory(trajectory2);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
