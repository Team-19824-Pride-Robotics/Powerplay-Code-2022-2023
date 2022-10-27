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
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
        telemetry.addData("claw1 pos",servo1.getPosition());
        telemetry.addData("claw2 pos",servo2.getPosition());
        telemetry.addData("arm pos",servo3.getPosition());
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
        double elevator_strength = .5;



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

        //elevator control
        else if (gamepad1.dpad_right) {
            leftRear.setPower(-d_power);
            leftFront.setPower(d_power);
            rightRear.setPower(d_power);
            rightFront.setPower(-d_power);
        }
        if (gamepad1.y) {
            elevator.setTargetPosition((int) top);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        //elevator to mid
        else if (gamepad1.x) {
            elevator.setTargetPosition((int) mid);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        //elevator to low
        else if (gamepad1.a) {
            elevator.setTargetPosition((int) low);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }
        // elevator to pickup
        else if (gamepad1.b) {
            //pickup = -365;
            servo3.setPosition(am);
            elevator.setTargetPosition((int) pickup);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(elevator_strength);
        }



        //gampad2

        //open claw
        else if(gamepad2.left_bumper) {
            servo1.setPosition(sr1o);
            servo2.setPosition(sr2o);
        }
        //close claw
        else if(gamepad2.right_bumper) {
            servo1.setPosition(sr1c);
            servo2.setPosition(sr2c);
        }
        //arm to left
        else if (gamepad2.dpad_left) {
            servo3.setPosition(al);
        }
        //arm to mid
        else if (gamepad2.dpad_up) {
            servo3.setPosition(am);
         }
        else if (gamepad2.dpad_right) {
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

    @Override
    public void stop() {

    }



}