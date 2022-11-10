package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="a_Nala_3_Auto")

//@Disabled
public class a_Nala_3_Auto extends LinearOpMode {

    public static double armMiddle = 0.38;
    public static int topCone = -650;
    public static int secondCone = -500;
    public static double parkY = 30;

    // to first pole
    public static double x1 = 60.6;
    public static double y1 = 1.5;
    //back up to line up for pickup
    public static double x2 = 47.22;
    public static double y2 = 0;
    //cone stack location
    public static double x3 = 48.23;
    public static double y3 = 24.3;
    //backup to score
    public static double x4 = 47.98;
    public static double y4 = -8.5;


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

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    elevator.setTargetPosition(-4200);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servo3.setPosition(0.73);
                })

                //time for the arm to stop swinging
                .waitSeconds(1)

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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    elevator.setTargetPosition(topCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //back up, turn, and then drive to cone stack
                .lineTo(new Vector2d(x2,y2))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(x3,y3))

                //grab top cone and then raise the elevator up before backing away
                .UNSTABLE_addTemporalMarkerOffset(0    , () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    elevator.setTargetPosition(-4200);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //time to grab the cone and raise elevator
                .waitSeconds(1)

                //drive to the high junction
                .lineTo(new Vector2d(x4,y4))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    servo3.setPosition(0.73);
                })

                //time for the arm to stop swinging
                .waitSeconds(1)

                //open claw and swing arm back to middle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servo1.setPosition(.5);
                    servo2.setPosition(.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    servo3.setPosition(armMiddle);
                })

                //time to score and then swing the arm back
                .waitSeconds(1)

                //lower the elevator to "second cone" position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    elevator.setTargetPosition(secondCone);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //drive back to the cone stack
                .lineTo(new Vector2d(x3,y3))

                //grab second cone and then raise the elevator up before backing away
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    servo1.setPosition(.68);
                    servo2.setPosition(.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(-4200);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //time to grab the cone and raise elevator
                .waitSeconds(2)

                //drive to the high junction
                .lineTo(new Vector2d(x4,y4))


                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    servo3.setPosition(0.73);
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
                .waitSeconds(.25)
                .strafeLeft(1)
                //time to score and then swing the arm back
                .waitSeconds(1)

                //lower the elevator to pickup position
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    elevator.setTargetPosition(-20);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                })

                //use the parkY variable to park in the correct zone
               // .forward(parkY)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        PoseStorage.currentPose = drive.getPoseEstimate();

    }

}