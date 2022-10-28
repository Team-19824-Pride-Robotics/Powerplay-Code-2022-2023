package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="SamplePractice3_Auto")
@Config
//@Disabled
public class SamplePractice3_Auto extends LinearOpMode {


    public static double sx = -69;
    public static double sy = -15;
    public static double sd = 90;
    public static double l1x = 0;
    public static double l1y = 0;
    public static double t1 = 90;
    public static double l2x = 0;
    public static double l2y = 0;
    public static double t2 = 45;
    public static double t3 = 135;
    public static double l3x = 0;
    public static double l3y = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(30)
//                .waitSeconds(2)
//                .turn(Math.toRadians(90))
//                .waitSeconds(2)
//                .strafeLeft(30)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}