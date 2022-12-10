/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.opmode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

//@Disabled
@Autonomous
public class April_Tag_Auto_Left extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;


    public static double armMiddle = 0.38;
    public static int topCone = -650;
    public static int secondCone = -500;
    public static double elevator_strength = 1;

    // to first pole
    public static double x1 = 39.36;
    public static double y1 = 2.5;
    //back up to line up for pickup
    public static double x2 = 48.5;
    public static double y2 = 2.5;
    //cone stack location
    public static double x3 = 48.5;
    public static double y3 = 23;
    //backup to score
    public static double x4 = 48.75;
    public static double y4 = -11.2;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

   // int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    //tag IDs of sleeve markers
    int park_left = 0;
    int park_middle = 1;
    int park_right = 2;

    //variable for parking
    double parkY = 30;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == park_left || tag.id == park_middle || tag.id == park_right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful*/
        if(tagOfInterest == null || tagOfInterest.id == park_left)
        {
           //set the parking variable to go to the left (or 1) position
            parkY = 30;
        }
        else if (tagOfInterest.id == park_middle)
        {
            //set the parking variable to go to the middle (or 2) position
            parkY = 10;
        }
        else if (tagOfInterest.id == park_right)
        {
            //set the parking variable to go to the right (or 3) position
            parkY = -13;
        }



        if (opModeIsActive()) {

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                    //close the claw
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(0.8);
                        servo2.setPosition(0.44);
                    })

                    //drive to high junction
                    .lineTo(new Vector2d(x1,y1))

                    //move arm up, then swing it into position (while driving)
                    .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                        elevator.setTargetPosition(-1850);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(.06);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(1)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(0.6);
                        servo2.setPosition(0.6);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    //time to score and then swing the arm back
                    .waitSeconds(1)

                    //lower the elevator to "top cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(topCone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //back up, turn, and then drive to cone stack
                    .lineTo(new Vector2d(x2,y2))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(x3,y3))

                    //grab top cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(-0.5    , () -> {
                        servo1.setPosition(0.8);
                        servo2.setPosition(0.44);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(-4000);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(0.5)

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
                        servo1.setPosition(0.6);
                        servo2.setPosition(0.6);
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
                        elevator.setPower(elevator_strength);
                    })

                    //drive back to the cone stack
                    .lineTo(new Vector2d(x3,y3))

                    //grab second cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        servo1.setPosition(0.8);
                        servo2.setPosition(0.44);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(-4000);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(0.5)

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
                        servo1.setPosition(0.6);
                        servo2.setPosition(0.6);
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
                        elevator.setPower(elevator_strength);
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

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
