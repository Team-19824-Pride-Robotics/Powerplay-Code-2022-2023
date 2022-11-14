package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class a_Nala_3_TeleOp extends LinearOpMode {

    public static double elevator_strength = 1;
    public static double sr1o = 0.5;
    public static double sr2o = 0.8;
    public static double sr1c = 0.65;
    public static double sr2c = 0.6;
    public static double al = .06;
    public static double am = 0.38;
    public static double ar = .73;
    public static double top = -4200;
    public static double mid = -3000;
    public static double low = -1850;
    public static double ground = -200;
    public static double pickup = -20;
    public static double x1 = 5.02;
    public static double y1 = -26.8;
    public static double x2 = 38.99;
    public static double y2 = -18.33;
    public static double h1 = 200.63;
    public static double h2 = 180;


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


        waitForStart();

        while (!isStopRequested()) {

            /*//////////////////////////
            DRIVER 1 CONTROLS START HERE
            *///////////////////////////
            double driving = (-gamepad1.left_stick_y) * 0.75;
            double strafing = (gamepad1.left_stick_x) * 0.75;
            double turning = (-gamepad1.right_stick_x) * 0.75;

            if(gamepad1.dpad_left) {
                strafing = -0.25;
            }
            if(gamepad1.dpad_right) {
                strafing = 0.25;
            }
            if(gamepad1.dpad_up) {
                driving = -0.25;
            }
            if(gamepad1.dpad_down) {
                driving = 0.25;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (driving),
                            (strafing),
                            (turning)
                    )
            );

            drive.update();

            if (gamepad1.a) {
                elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator.setPower(-gamepad2.right_stick_y);
            }

            if (gamepad1.y) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
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
                        .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)))
                        .addDisplacementMarker(1, () -> {
                            servo3.setPosition(am);
                            elevator.setTargetPosition((int) top);
                            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            elevator.setPower(elevator_strength);

                        })
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }


            /*//////////////////////////
            DRIVER 2 CONTROLS START HERE
            *///////////////////////////

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
            if (gamepad2.dpad_down) {
                elevator.setTargetPosition((int) ground);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            if (gamepad2.y) {
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            if (gamepad2.x) {
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            if (gamepad2.a) {
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            if (gamepad2.b) {
                servo3.setPosition(am);
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
        }
    }
}