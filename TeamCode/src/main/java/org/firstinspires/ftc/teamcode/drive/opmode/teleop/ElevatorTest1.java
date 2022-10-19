package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.SimpleDateFormat;
import java.util.Date;


@TeleOp
@Config

public class ElevatorTest1 extends OpMode {
    /* Declare OpMode members. */
    //private Blinker control_Hub;
    // private Blinker expansion_Hub;
    private DcMotor elevator;
    private Gyroscope imu;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    public static double sr1o = .5;
    public static double sr2o = .8;
    public static double sr1c = .68;
    public static double sr2c = 1.6;




    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime     runtime = new ElapsedTime();

    double drive;
    double strafe;
    double rotate;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //   control_Hub = hardwareMap.get(Blinker.class, "Control_Hub");
        //   expansion_Hub = hardwareMap.get(Blinker.class, "Expansion_Hub");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftRear = hardwareMap.get(DcMotor.class, "LB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightRear = hardwareMap.get(DcMotor.class, "RB");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
        telemetry.update();
        double d_power = .8-.4*gamepad1.left_trigger+(.5*gamepad1.right_trigger);
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate_stick = gamepad1.right_stick_x;
        double rotate_button = 0;
        //CLOSE THE CLAW!!


        if (gamepad1.left_bumper) {
            rotate_button = -d_power;
        }
        else if (gamepad1.right_bumper) {
            rotate_button = d_power;
        }
        else {
            rotate_button = 0;
        }
        rotate = rotate_stick + .5*rotate_button;

        leftRear.setPower(drive + rotate);
        leftFront.setPower(drive + rotate);
        rightRear.setPower(drive - rotate);
        rightFront.setPower(drive - rotate);



        if (gamepad1.dpad_up) {
            leftRear.setPower(d_power);
            leftFront.setPower(d_power);
            rightRear.setPower(d_power);
            rightFront.setPower(d_power);
        }
        else if (gamepad1.dpad_down) {
            leftRear.setPower(-d_power);
            leftFront.setPower(-d_power);
            rightRear.setPower(-d_power);
            rightFront.setPower(-d_power);
        }
        else if (gamepad1.dpad_left) {
            leftRear.setPower(d_power);
            leftFront.setPower(-d_power);
            rightRear.setPower(-d_power);
            rightFront.setPower(d_power);
        }
        else if (gamepad1.b) {
            leftRear.setPower(-d_power);
            leftFront.setPower(d_power);
            rightRear.setPower(d_power);
            rightFront.setPower(-d_power);
        }
        else if (gamepad1.x) {
            leftRear.setPower(d_power);
            leftFront.setPower(-d_power);
            rightRear.setPower(-d_power);
            rightFront.setPower(d_power);
        }
        else if (gamepad1.dpad_right) {
            leftRear.setPower(-d_power);
            leftFront.setPower(d_power);
            rightRear.setPower(d_power);
            rightFront.setPower(-d_power);
        }


        double elevator_strength = .5;
        int top = -3775;
        int mid = -2655;
        int low = -1635;
        int pickup = 0;

        if (gamepad2.dpad_left) {
            elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator.setPower(-gamepad2.right_stick_y);
        }

        else if (gamepad2.y) {
            elevator.setTargetPosition(top);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        else if (gamepad2.x) {
            elevator.setTargetPosition(mid);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        else if (gamepad2.a) {
            elevator.setTargetPosition(low);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        else if (gamepad2.b) {
            elevator.setTargetPosition(pickup);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        else if(gamepad2.left_bumper) {
            servo1.setPosition(sr1o);
            servo2.setPosition(sr2o);
        }
        //OPEN THE CLAW!!
        else if(gamepad2.right_bumper) {
            servo1.setPosition(sr1c);
            servo2.setPosition(sr2c);

        }





    }

    @Override
    public void stop() {

    }



}