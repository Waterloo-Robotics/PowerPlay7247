package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;

@TeleOp(name = "Blue Alliance Red Corner")
public class BaRc extends LinearOpMode {

    double turn1 = 104;
    double strafey = 3;

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory strafe1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, strafey, 0))
                .build();

        Trajectory moveForward1 = drive.trajectoryBuilder(strafe1.end())
                .lineToLinearHeading(new Pose2d(48, strafey, 0))
                .build();

        Trajectory moveForward2 = drive.trajectoryBuilder(new Pose2d(48, 4, Math.toRadians(turn1)))
                .lineToLinearHeading(new Pose2d(62, strafey, Math.toRadians(turn1)))
                .build();

        Trajectory moveToBlack = drive.trajectoryBuilder(moveForward2.end())
                .lineToLinearHeading(new Pose2d(59.5, 15, Math.toRadians(turn1)))
                .build();

        drive.followTrajectory(strafe1);

        drive.followTrajectory(moveForward1);

        drive.turn(Math.toRadians(turn1));

        drive.followTrajectory(moveForward2);

        drive.followTrajectory(moveToBlack);

        Pose2d poseEstimate;

        while (!isStopRequested() && !gamepad1.a) {

            drive.update();

            poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }

    }

}
