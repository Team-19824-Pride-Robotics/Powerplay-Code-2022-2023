package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.samples.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class practice3_TeleOp_copy extends LinearOpMode {


    //////////////Variables we might change at some point
    public static double sr1o = 0.5;
    public static double sr2o = 0.8;
    public static double sr1c = 0.68;
    public static double sr2c = 0.6;
    public static double al = 0.02;
    public static double am = 0.35;
    public static double ar = 0.69;
    public static double top = -4200;
    public static double mid = -3200;
    public static double low = -1900;
    public static double pickup = -365;
    public static double cup = 50;
    double slow = 0.25;
    double fromWall = 5;
    double elevator_strength = .5;

//////////Motor setup for the ones that aren't drive motors
//////////(drive motors are set up in "Sample Mecanum Drive")
    private DcMotor elevator;
    private Gyroscope imu;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        DistanceSensor distance;
//        distance = hardwareMap.get(DistanceSensor.class, "frontDist");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        elevator = hardwareMap.get(DcMotor.class, "elevator");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (!isStopRequested()) {


            telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
            telemetry.addData("claw1 pos",servo1.getPosition());
            telemetry.addData("claw2 pos",servo2.getPosition());
            telemetry.addData("arm pos",servo3.getPosition());
            telemetry.update();


///////////////this code drives the robot using the roadrunner codebase
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

//            if (gamepad1.x) {
//                drive.turn(Math.toRadians(180));
//            }
//            if (gamepad1.dpad_up && fromWall < 12) {
//                fromWall += 1;
//                sleep(200);
//            }
//            if (gamepad1.dpad_down && fromWall > 0) {
//                fromWall -= 1;
//                sleep(200);
//            }
            drive.update();

/////////////////here is the code to run our mechanisms



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            //telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("fromWall", fromWall);
            telemetry.update();


/////////////////ELEVATOR CONTROLS
            //elevator to highest junction level
            if (gamepad1.y) {
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            //elevator to middle junction level
            if (gamepad1.x) {
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            //elevator to lowest junction level
            if (gamepad1.a) {
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            // elevator to pickup position
            if (gamepad1.b) {
                //pickup = -365;
                servo3.setPosition(am);
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

//////////////////////CLAW AND SWING ARM CONTROLS

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
            if (gamepad2.dpad_left) {
                servo3.setPosition(al);
            }
            //arm to mid
            if (gamepad2.dpad_up) {
                servo3.setPosition(am);
            }
            //arm to right
            if (gamepad2.dpad_right) {
                servo3.setPosition(ar);
            }
            if (gamepad2.y) {
                pickup -= cup;
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);

            }
            if (gamepad2.a) {
                pickup += cup;
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }


        }
    }
}
