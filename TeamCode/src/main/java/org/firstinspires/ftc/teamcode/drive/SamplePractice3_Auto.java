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


@Autonomous(name="SamplePractice3_Auto")

//@Disabled
public class SamplePractice3_Auto extends LinearOpMode {





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
                .lineTo(new Vector2d(39.65,0))
                //move arm up
                .addTemporalMarker(0, () -> {
                    elevator.setTargetPosition(-3950);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .waitSeconds(3)
                //move arm to top pole
                .addTemporalMarker(0, () -> {
                    servo3.setPosition(0.69);
                })
                .waitSeconds(3)
                //open claw
                .addTemporalMarker(0, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(57.9,0))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(57.9,16.85))
                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}