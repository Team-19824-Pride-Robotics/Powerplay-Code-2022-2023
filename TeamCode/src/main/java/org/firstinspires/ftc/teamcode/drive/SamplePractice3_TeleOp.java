package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class SamplePractice3_TeleOp extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor elevator;
        Gyroscope imu;
        Servo servo1;
        Servo servo2;
        Servo servo3;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double slow = 0.25;
        double fromWall = 5;
        double elevator_strength = .5;
        double sr1o = 0.5;
        double sr2o = 0.8;
        double sr1c = 0.68;
        double sr2c = 0.6;
        double al = 0.02;
        double am = 0.35;
        double ar = 0.69;
        double top = -4200;
        double mid = -3200;
        double low = -1900;
        double pickup = -365;
        double cup = 50;

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.right_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y*slow,
                                -gamepad1.left_stick_x*slow,
                                -gamepad1.right_stick_x*slow
                        )
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.x) {
                drive.turn(Math.toRadians(180));
            }
            if (gamepad1.dpad_up && fromWall < 12) {
                fromWall += 1;
                sleep(200);
            }
            if (gamepad1.dpad_down && fromWall > 0) {
                fromWall -= 1;
                sleep(200);
            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            //telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("fromWall", fromWall);
            telemetry.update();

            //open claw
            if(gamepad2.left_bumper) {
                servo1.setPosition(sr1o);
                servo2.setPosition(sr2o);
            }
            //close claw
            if(gamepad2.right_bumper) {
                servo1.setPosition(sr1c);
                servo2.setPosition(sr2c);
            }



            if (gamepad1.a) {
                //double dist = distance.getDistance(DistanceUnit.INCH);

                /*
                method 1 uses the pose estimate, you can increase or decrease "fromWall"
                 to drive that distance from the wall (assuming the wall is x = 0)
                dpad up/down can be used to change the "fromWall" distance in a match
                */

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        // .forward(poseEstimate.getX() - fromWall)
                        .strafeLeft(poseEstimate.getX() - fromWall)
                        //.turn(Math.toRadians(180))
                        .build();

                /*
                method 2 works the same but uses the distance sensor
                */
//                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .forward(dist - fromWall)
//                        //.turn(Math.toRadians(180))
//                        .build();


                drive.followTrajectorySequence(trajSeq);

            }
        }
    }

}