package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="a_3_alaN_Auto_Final_Left")

//@Disabled
public class a_3_alaN_Auto_Final_Left extends LinearOpMode {

    public static double armMiddle = 0.38;
    public static int topCone = -650;
    public static int secondCone = -500;
    public static double parkY = 32;

    // to first pole
    public static double x1 = 59.5;
    public static double y1 = 0;
    //back up to line up for pickup
    public static double x2 = 47;
    public static double y2 = 0;
    //cone stack location
    public static double x3 = 47.5;
    public static double y3 = 23;
    //backup to score
    public static double x4 = 47.5;
    public static double y4 = -11.5;


    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
     //private static final String TFOD_MODEL_ASSET  = "powerplay_19824_signal.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "ATikKv7/////AAABmZEp8imS2kWSho7r5LLtLJ8aIM3qubW0EhIhJbicruc1o7KMC59JuCMVgBYtWWPQmuIcP1orD6ULF8fDXHrt1+efUVLBiAlMnhdv0PFhCbIacnvbZyiw2xgbwH6E+fSpZuEzJGphHvkW2RXDMLmbrhCpHdGOK6zN75R8o5ZQ5JyGinVvFF8Y7/4tCEiqPH+cNiv4xtX3TthzBWj6CzZGreirC9SjAJLdXJO9WwTV7y0Yoh0IntB5CdC9EMrr+koPH5h0RjOv5qC3KgP6ulmLYQJLVeHUyZhFQuGuO/Wu0K3/ARiOlF+jYWyU7yP5unMJAF3BfnFBN1B0/3OIAos1zKprKMiN69RrGNckA0xK8IFd";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        DcMotor elevator;
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

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        waitForStart();

                    for (int i = 0; i < 20; i++) {

                if (tfod != null) {


                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            if(recognition.getLabel() == "2 Bulb") {
                                parkY = 10;
                            }
                            if(recognition.getLabel() == "3 Panel") {
                                parkY = -13;
                            }
                            if(recognition.getLabel() == "1 Bolt") {
                                parkY = 30;
                            }
                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            telemetry.addData("Parking Location: ", parkY);

                        }
                        telemetry.update();
                    }
                }
            }

        if (opModeIsActive()) {


            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                    //close the claw
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(.65);
                        servo2.setPosition(.6);
                    })

                    //drive to high junction
                    .lineTo(new Vector2d(x1,y1))

                    //move arm up, then swing it into position (while driving)
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        elevator.setTargetPosition(-4150);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        servo3.setPosition(0.73);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(1)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(.5);
                        servo2.setPosition(.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    //time to score and then swing the arm back
                    .waitSeconds(2)

                    //lower the elevator to "top cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(topCone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })

                    //back up, turn, and then drive to cone stack
                    .lineTo(new Vector2d(x2,y2))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(x3,y3))

                    //grab top cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(0    , () -> {
                        servo1.setPosition(.65);
                        servo2.setPosition(.6);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        elevator.setTargetPosition(-4150);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(1)

                    //drive to the high junction
                    .lineTo(new Vector2d(x4,y4))

                    //swing the arm to the right while driving
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(0.73);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(1)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(.5);
                        servo2.setPosition(.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    //time to score and then swing the arm back
                    .waitSeconds(2)

                    //lower the elevator to "second cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(secondCone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })

                    //drive back to the cone stack
                    .lineTo(new Vector2d(x3,y3))

                    //grab second cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        servo1.setPosition(.65);
                        servo2.setPosition(.6);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
                        elevator.setTargetPosition(-4150);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(1.5)

                    //drive to the high junction
                    .lineTo(new Vector2d(x4,y4))

                    //swing the arm to the right while driving
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(0.73);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(2)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(.5);
                        servo2.setPosition(.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        servo3.setPosition(armMiddle);
                    })
                    //time to score and then swing the arm back
                    .waitSeconds(2)

                    //lower the elevator to pickup position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(-20);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(1);
                    })

                    //use the parkY variable to park in the correct zone
                    .forward(parkY)
                    .build();

            if (!isStopRequested()) {
                drive.followTrajectorySequence(trajSeq);
            }

            PoseStorage.currentPose = drive.getPoseEstimate();

        }
    }

    /**
            * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}