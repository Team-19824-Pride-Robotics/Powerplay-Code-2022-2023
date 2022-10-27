package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Teleop22_23v1.ElevatorLevel.GROUND;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "drive")
public class Teleop22_23v1 extends LinearOpMode {
    //enumerator to keep track of elevator levels
    enum ElevatorLevel {
        GROUND,
        SHORT,
        MEDIUM,
        TALL
    }

    @Override

    public void runOpMode() throws InterruptedException {
        //variables
        DcMotor elevator = hardwareMap.get(DcMotor.class, "elevator");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slow = 0.25;
        ElevatorLevel level = GROUND;

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.right_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * slow,
                                -gamepad1.left_stick_x * slow,
                                -gamepad1.right_stick_x * slow
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad2.right_trigger > 0) {
                /*raise elevator until it hits max height or trigger is no longer held dwn
                 * */
                elevator.setPower(1);
            }


            if (gamepad2.left_trigger > 0) {
                elevator.setPower(-0.1);
            }


            //moves elevator up incrementally to each level
            if (gamepad2.right_bumper) {
                //encoder values are placeholders until we know what the actual values are
                if (level == ElevatorLevel.GROUND) {
                    //if elevator level is ground, move to short
                    elevator.setTargetPosition(1000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                    level = ElevatorLevel.SHORT;
                } else if (level == ElevatorLevel.SHORT) {
                    //if elevator level is short move to medium
                    elevator.setTargetPosition(2000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                    level = ElevatorLevel.MEDIUM;
                } else if (level == ElevatorLevel.MEDIUM) {
                    //if elevator level is medium move to tall
                    elevator.setTargetPosition(3000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                } else {
                    //do nothing
                }
            }

            //moves elevator down incrementally to each level

            if (gamepad2.left_bumper) {
                //encoder values are placeholders until we know what the actual values are
                if (level == ElevatorLevel.TALL) {
                    //if elevator level is tall, move to medium
                    elevator.setTargetPosition(2000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                    level = ElevatorLevel.MEDIUM;
                } else if (level == ElevatorLevel.MEDIUM) {
                    //if elevator level is medium move to short
                    elevator.setTargetPosition(1000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                    level = ElevatorLevel.SHORT;
                } else if (level == ElevatorLevel.SHORT) {
                    //if elevator level is short move to ground
                    elevator.setTargetPosition(0);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.25);
                    level = ElevatorLevel.GROUND;
                } else {
                    //do nothing
                }
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
