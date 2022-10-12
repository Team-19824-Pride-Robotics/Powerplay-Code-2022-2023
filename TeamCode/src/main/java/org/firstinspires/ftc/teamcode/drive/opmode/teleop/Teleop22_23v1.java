package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.samples.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "drive")
public class Teleop22_23v1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DistanceSensor distance;
        distance = hardwareMap.get(DistanceSensor.class, "frontDist");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slow = 0.25;
        double fromWall = 5;

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
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("fromWall", fromWall);
            telemetry.update();

            if (gamepad1.a) {
                double dist = distance.getDistance(DistanceUnit.INCH);

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
