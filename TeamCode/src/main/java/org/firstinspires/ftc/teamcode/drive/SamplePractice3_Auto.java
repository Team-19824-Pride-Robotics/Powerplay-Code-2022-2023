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
    public static int topCone = -600;
    public static int secondCone = -400;
    public static double parkY = 30;

    // to first pole
    public static double x1 = 60.6;
    public static double y1 = 1.5;
    //back up to line up for pickup
    public static double x2 = 47.22;
    public static double y2 = 0;
    //cone stack location
    public static double x3 = 48.23;
    public static double y3 = 24.8;
    //backup to score
    public static double x4 = 47.98;
    public static double y4 = -7.38;


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
                //close the claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })

                //drive to high junction
                .lineTo(new Vector2d(x1,y1))

                //move arm up, then swing it into position (while driving)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    elevator.setTargetPosition(-4000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servo3.setPosition(0.74);
                })

                //time for the arm to stop swinging
                .waitSeconds(2)

                //open claw and swing arm back to middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo3.setPosition(armMiddle);
                })

                //time to score and then swing the arm back
                .waitSeconds(2)

                //lower the elevator to "top cone" position
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    elevator.setTargetPosition(topCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //back up, turn, and then drive to cone stack
                .lineTo(new Vector2d(x2,y2))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(x3,y3))

                //grab top cone and then raise the elevator up before backing away
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(-1200);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //time to grab the cone and raise elevator
                .waitSeconds(3)

                //drive to the high junction
                .lineTo(new Vector2d(x4,y4))

                //move arm up, then swing it into position (while driving)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    elevator.setTargetPosition(-4000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servo3.setPosition(0.74);
                })

                //time for the arm to stop swinging
                .waitSeconds(2)

                //open claw and swing arm back to middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo3.setPosition(armMiddle);
                })

                //time to score and then swing the arm back
                .waitSeconds(2)

                //lower the elevator to "second cone" position
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    elevator.setTargetPosition(secondCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //drive back to the cone stack
                .lineTo(new Vector2d(x3,y3))

                //grab top cone and then raise the elevator up before backing away
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(-1200);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //time to grab the cone and raise elevator
                .waitSeconds(3)

                //drive to the high junction
                .lineTo(new Vector2d(x4,y4))

                //move arm up, then swing it into position (while driving)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    elevator.setTargetPosition(-4000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servo3.setPosition(0.74);
                })

                //time for the arm to stop swinging
                .waitSeconds(2)

                //open claw and swing arm back to middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo3.setPosition(armMiddle);
                })

                //time to score and then swing the arm back
                .waitSeconds(2)

                //lower the elevator to pickup position
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    elevator.setTargetPosition(-20);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //use the parkY variable to park in the correct zone
                .forward(parkY)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}