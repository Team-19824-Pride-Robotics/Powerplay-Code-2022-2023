package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="SamplePractice3_Auto")

//@Disabled
public class SamplePractice3_Auto extends LinearOpMode {

    public static double armMiddle = 0.38;

    // to first pole
    public static double x1 = 60.6;
    public static double y1 = 1.5;
    //back up to line up for pickup
    public static double x2 = 47.22;
    public static double y2 = 0;
    //pickup
    public static double x3 = 48.23;
    public static double y3 = 24.8;
    //backup to score
    public static double x4 = 47.98;
    public static double y4 = -7.38;
    //forward to pickup
    public static double x5 = 48.23;
    public static double y5 = 24.8;
    public static int topCone = -600;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        DcMotor elevator;
        Servo servo1;
        Servo servo2;
        Servo servo3;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                //.waitSeconds(1)
                .lineTo(new Vector2d(x1,y1))
                //move arm up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    elevator.setTargetPosition(-4000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                //swing arm to top pole
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    servo3.setPosition(0.74);
                })
                //open claw
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                //wait 2 additional seconds
                .waitSeconds(2)
                .lineTo(new Vector2d(x2,y2))
                //move the arm to the middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo3.setPosition(armMiddle);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    elevator.setTargetPosition(topCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(x3,y3))
               // grab cone
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .waitSeconds(3)
                .lineTo(new Vector2d(x4,y4))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(-4000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    servo3.setPosition(0.74);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                //wait an additional 2 seconds
                .waitSeconds(7)
                .lineTo(new Vector2d(x5,y5))
                //move the arm to the middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo3.setPosition(armMiddle);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    elevator.setTargetPosition(topCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}