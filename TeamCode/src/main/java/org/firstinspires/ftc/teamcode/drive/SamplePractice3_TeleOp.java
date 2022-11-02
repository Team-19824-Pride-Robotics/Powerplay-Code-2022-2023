package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class SamplePractice3_TeleOp extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);

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
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double slow = 0.25;
        double fromWall = 5;
        double elevator_strength = 1;
        double sr1o = 0.5;
        double sr2o = 0.8;
        double sr1c = 0.68;
        double sr2c = 0.6;
        double al = 0.02;
        double am = 0.35;
        double ar = 0.69;
        double top = -4200;
        double mid = -2900;
        double low = -1800;
        double pickup = 0;
        double side = -500;

        servo1.setPosition(sr1c);
        servo2.setPosition(sr2c);
        servo3.setPosition(am);


        waitForStart();

        while (!isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (gamepad1.left_stick_y),
                            (gamepad1.left_stick_x),
                            (-gamepad1.right_stick_x)
                    )
            );


            if (gamepad1.dpad_down) {
                drive.turn(Math.toRadians(180));
            }
//            if (gamepad1.dpad_up && fromWall < 12) {
//                fromWall += 1;
//                sleep(200);
//            }
//            if (gamepad1.dpad_down && fromWall > 0) {
//                fromWall -= 1;
//                sleep(200);
//            }


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            //telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("fromWall", fromWall);
            telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
            telemetry.addData("claw1 pos",servo1.getPosition());
            telemetry.addData("claw2 pos",servo2.getPosition());
            telemetry.addData("arm pos",servo3.getPosition());
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
            //arm to left
            if (gamepad2.dpad_right) {
                servo3.setPosition(al);
            }
            //arm to mid
            if (gamepad2.dpad_up) {
                servo3.setPosition(am);
            }
            //arm to right
            if (gamepad2.dpad_left) {
                servo3.setPosition(ar);
            }
            if (gamepad2.y) {
                //pickup -= cup;
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);

            }

            if (gamepad2.x) {
                //pickup += cup;
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad2.a) {
                //pickup += cup;
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad2.b) {
                //pickup += cup;
                servo3.setPosition(am);
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad1.dpad_up) {
                side -= 50;
                elevator.setTargetPosition((int) side);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad1.dpad_down) {
                side += 50;
                elevator.setTargetPosition((int) side);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad1.dpad_right) {
                elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator.setPower(-gamepad2.right_stick_y);
            }

            if (gamepad1.y) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(5.02, -26.8, Math.toRadians(200.63)))
                        .addTemporalMarker(0, () -> {
                            servo3.setPosition(am);
                            elevator.setTargetPosition((int) pickup);
                            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            elevator.setPower(elevator_strength);
                        })
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }
            if (gamepad1.x) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(38.99, -18.33, Math.toRadians(180)))
                        .addDisplacementMarker(1, () -> {
                            servo3.setPosition(am);
                            elevator.setTargetPosition((int) top);
                            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            elevator.setPower(elevator_strength);

                        })
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }

            if (gamepad1.a) {
                //double dist = distance.getDistance(DistanceUnit.INCH);

                /*
                method 1 uses the pose estimate, you can increase or decrease "fromWall"
                 to drive that distance from the wall (assuming the wall is x = 0)
                dpad up/down can be used to change the "fromWall" distance in a match
                */

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        //.forward(10)
                        //.strafeLeft(poseEstimate.getX() - fromWall)
                        //.turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(0.2, -19.04, Math.toRadians(193.53)))
                        .build();

                drive.followTrajectorySequenceAsync(trajSeq);
//
//                /*
//                method 2 works the same but uses the distance sensor
//                */
////                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                        .forward(dist - fromWall)
////                        //.turn(Math.toRadians(180))
////                        .build();
//
//
//                drive.followTrajectorySequence(trajSeq);
//
            }
            if(gamepad1.b) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        //.forward(10)
                        //.strafeLeft(poseEstimate.getX() - fromWall)
                        //.turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(34.28, -20.9, Math.toRadians(180)))
                        .build();

                drive.followTrajectorySequenceAsync(trajSeq);
            }
        }
    }


}