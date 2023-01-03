package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;
import encoder.odo.ftc.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveToCone = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(24, 0))
                .build();

//        Trajectory moveToCone1 = drive.trajectoryBuilder(moveToCone.end())
//                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(5)))
//                .build();

        waitForStart();

        drive.followTrajectory(moveToCone);
//        drive.followTrajectory(moveToCone1);

    }

}
