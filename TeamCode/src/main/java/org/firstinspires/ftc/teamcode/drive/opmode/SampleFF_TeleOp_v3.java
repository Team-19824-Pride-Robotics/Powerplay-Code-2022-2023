package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class SampleFF_TeleOp_v3 extends OpMode {
    /* Declare OpMode members. */
    private Blinker control_Hub;
    private Blinker expansion_Hub;
    private DcMotor armDrive;
    private DcMotor intake;
    private Gyroscope imu;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor carousel;
    private DcMotor rightFront;
    private DcMotor rightRear;

    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime     runtime = new ElapsedTime();

    double drive;
    double strafe;
    double rotate;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        control_Hub = hardwareMap.get(Blinker.class, "Control_Hub");
        expansion_Hub = hardwareMap.get(Blinker.class, "Expansion_Hub");
        armDrive = hardwareMap.get(DcMotor.class, "armDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Encoder ld", leftRear.getCurrentPosition());
        telemetry.addData("Encoder ld2", leftFront.getCurrentPosition());
        telemetry.addData("Encoder rd", rightRear.getCurrentPosition());
        telemetry.addData("Encoder rd2", rightFront.getCurrentPosition());
        telemetry.addData("Encoder ad", armDrive.getCurrentPosition());
        telemetry.addData("Encoder ltbd", carousel.getCurrentPosition());
        telemetry.addData("Encoder fs2", intake.getCurrentPosition());
        telemetry.addData("Carousel", gamepad2.left_stick_y);
        telemetry.update();
        double d_power = .8-.4*gamepad1.left_trigger+(.5*gamepad1.right_trigger);
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate_stick = gamepad1.right_stick_x;
        double rotate_button = 0;

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
        //   else if (gamepad1.y) {
        //     drive(0.5, 21.5, -21.5, -21.5, 21.5);
        //     drive(1, 30, 30, 30, 30);
        //      //switch to run to position mode
        //   leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //   leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //   rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //   rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //   }

        double arm_strength = .5;
        int over = -80;
        int upper = -1065;
        int cap = -1200;
        int lower = -1417;
        int pickup = -1617;

        if (gamepad2.b) {
            armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armDrive.setPower(-gamepad2.right_stick_y);
        }
        //carousel.setPower(-2*gamepad2.right_stick_y);

        else if (gamepad2.y) {
            armDrive.setTargetPosition(over);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(0.25);
        }
        else if (gamepad2.dpad_up) {
            armDrive.setTargetPosition(-855);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(0.25);
        }
        else if (gamepad2.x) {
            armDrive.setTargetPosition(-1016);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(0.25);
        }
        else if (gamepad2.dpad_down) {
            armDrive.setTargetPosition(-1406);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(arm_strength);
        }
        else if (gamepad2.dpad_right) {
            armDrive.setTargetPosition(-1272);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(arm_strength);
        }
        else if (gamepad2.a) {
            armDrive.setTargetPosition(pickup);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(arm_strength);
        }
        else {
            armDrive.setTargetPosition(upper);
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive.setPower(arm_strength);
        }

        if (gamepad2.left_trigger > .1) {
            intake.setPower(.9*gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger > .1) {
            intake.setPower(-.6*gamepad2.right_trigger);
        }
        else {
            intake.setPower(0);
        }

        if (gamepad2.left_bumper) {
            carousel.setPower(0.24);
            if (gamepad2.left_stick_y < -0.1) {
                carousel.setPower(1);
            }
            else if (gamepad2.left_stick_y > 0.1) {
                carousel.setPower(-0.1);
            }
        }

        else if (gamepad2.right_bumper) {
            carousel.setPower(-0.24);
            if (gamepad2.left_stick_y < -0.1) {
                carousel.setPower(-1);
            }
            else if (gamepad2.left_stick_y > 0.1){
                carousel.setPower(0.1);
            }
        }
        else {
            carousel.setPower(0);
        }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

//a function to spin the carousel slow, then fast at the press of a button

    public void duck(double dir) {
        runtime.reset();
        if (runtime.seconds() < 2) {
            carousel.setPower(dir*0.5);
        }
        else if(runtime.seconds() < 4) {
            carousel.setPower(dir);
        }
        else {
            carousel.setPower(0);
        }
    }
}