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
// to first pole
    public static double x1 = 62.25;
    public static double y1 = 3;
    //back up to line up for pickup
    public static double x2 = 48.5;
    public static double y2 = 0;
    //pickup
    public static double x3 = 48;
    public static double y3 = 17;
    //backup to score
    public static double x4 = 48.5;
    public static double y4 = 0;
    //forward to pickup
    public static double x5 = 48;
    public static double y5 = 17;
// 4&5 vaule need to be set



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
                .addTemporalMarker(0, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(x1,y1))
                //move arm up
                .addTemporalMarker(2, () -> {
                    elevator.setTargetPosition(-3950);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                //move arm to top pole
                .addTemporalMarker(3, () -> {
                    servo3.setPosition(0.69);
                })
                //open claw
                .addTemporalMarker(6, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(x2,y2))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo3.setPosition(.35);
                    elevator.setTargetPosition(0);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(x3,y3))
               // grab cone
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .lineTo(new Vector2d(x4,y4))
                .addTemporalMarker(0, () -> {
                    elevator.setTargetPosition(-3950);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    servo3.setPosition(0.69);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .lineTo(new Vector2d(x5,y5))
                .addTemporalMarker(0, () -> {
                    servo3.setPosition(.35);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(0);
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