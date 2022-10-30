package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import encoder.odo.ftc.rr.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Alliance Red Corner")
public class BaRc extends LinearOpMode {

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory moveForward1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(48, 0, 0))
                .build();

        Trajectory moveForward2 = drive.trajectoryBuilder(new Pose2d(48, 0, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(60, 0, Math.toRadians(120)))
                .build();

        drive.followTrajectory(moveForward1);
        drive.turn(Math.toRadians(120));
        drive.followTrajectory(moveForward2);

    }

}
