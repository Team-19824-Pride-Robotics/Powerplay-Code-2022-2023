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
    private DcMotor elevator;
    @Override
    public void runOpMode() throws InterruptedException {
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slow = 0.25;

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

            if (gamepad2.right_trigger>0.5) {
                /*raise elevator until it hits max height or trigger is no longer held dwn
                 * */
                elevator.setPower(1);
            }

            if (gamepad2.right_bumper) {
                /*Raise to a set height when pressed small_pole==?, medium=?, large=?  */
            }



            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();



        }
    }

}