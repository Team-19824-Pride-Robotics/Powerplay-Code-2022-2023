//package org.firstinspires.ftc.teamcode.drive;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Gyroscope;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.opmode.samples.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//
//@TeleOp(group = "drive")
//@Config
//public class practice3_TeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
////        DistanceSensor distance;
////        distance = hardwareMap.get(DistanceSensor.class, "frontDist");
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
// //////////////Variables we might change at some point
//        public static double sr1o = 0.5;
//        public static double sr2o = 0.8;
//        public static double sr1c = 0.68;
//        public static double sr2c = 0.6;
//        public static double al = 0.02;
//        public static double am = 0.35;
//        public static double ar = 0.69;
//        public static double top = -4200;
//        public static double mid = -3200;
//        public static double low = -1900;
//        public static double pickup = -365;
//        public static double cup = 50;
//        double slow = 0.25;
//        double fromWall = 5;
//
////////////Motor setup for the ones that aren't drive motors
////////////(drive motors are set up in "Sample Mecanum Drive")
//        private DcMotor elevator;
//        private Gyroscope imu;
//        private Servo servo1;
//        private Servo servo2;
//        private Servo servo3;
//
//        waitForStart();
//
//        while (!isStopRequested()) {
//
//
//            if (gamepad1.right_bumper) {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y*slow,
//                                -gamepad1.left_stick_x*slow,
//                                -gamepad1.right_stick_x*slow
//                        )
//                );
//            }
//            else {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x,
//                                -gamepad1.right_stick_x
//                        )
//                );
//            }
//
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
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("fromWall", fromWall);
//            telemetry.update();
//
//            if (gamepad1.a) {
//                double dist = distance.getDistance(DistanceUnit.INCH);
//
//                /*
//                method 1 uses the pose estimate, you can increase or decrease "fromWall"
//                 to drive that distance from the wall (assuming the wall is x = 0)
//                dpad up/down can be used to change the "fromWall" distance in a match
//                */
//
//                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                       // .forward(poseEstimate.getX() - fromWall)
//                        .strafeLeft(poseEstimate.getX() - fromWall)
//                        //.turn(Math.toRadians(180))
//                        .build();
//
//                /*
//                method 2 works the same but uses the distance sensor
//
//                */
////                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                        .forward(dist - fromWall)
////                        //.turn(Math.toRadians(180))
////                        .build();
//
//
//                drive.followTrajectorySequence(trajSeq);
//
//            }
//        }
//    }
//}
