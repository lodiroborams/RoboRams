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

package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class Camera extends LinearOpMode
{

    private DcMotor FrontleftMotor;
    private DcMotor FrontrightMotor;
    private DcMotor BackleftMotor;
    private DcMotor BackrightMotor;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int tics = 1440;
    int revolution = 16;
    int inch = tics / revolution;

    static final double FEET_PER_METER = 3.28084;

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

    //Tag IDs
    int BlueLeft = 1;
    int BlueMiddle = 2;
    int BlueRight = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        FrontleftMotor = hardwareMap.get(DcMotor.class, "front_left");
        FrontrightMotor = hardwareMap.get(DcMotor.class, "front_right");
        BackleftMotor = hardwareMap.get(DcMotor.class, "back_left");
        BackrightMotor = hardwareMap.get(DcMotor.class, "back_right");

        FrontleftMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontrightMotor.setDirection(DcMotor.Direction.REVERSE);
        BackleftMotor.setDirection(DcMotor.Direction.FORWARD);
        BackrightMotor.setDirection(DcMotor.Direction.REVERSE);


        FrontleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontleftMotor.setPower(0);
        BackleftMotor.setPower(0);
        FrontrightMotor.setPower(0);
        BackrightMotor.setPower(0);


        FrontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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
        {}

        moveFoward(30,1);

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == BlueLeft || tag.id == BlueMiddle || tag.id == BlueRight)
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

        /* Actually do something useful */
       if(tagOfInterest == null){

           moveFoward(55,1);


       }else if (tagOfInterest.id == BlueLeft) {

           strafeleft(30,1);

       //blueleft code
       }else if (tagOfInterest == null || tagOfInterest.id == BlueMiddle) {

           strafeleft(30,1);
           moveFoward(35,1);
           //blue middle code
       }else if (tagOfInterest.id == BlueRight){

       }
           //Blue Right code




        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
    private void strafeleft (int distance, double power) {

        //int power = 0;
        FrontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FrontleftMotor.setTargetPosition(distance * inch); // check movements of the motors for negative and positive values
        FrontrightMotor.setTargetPosition(-distance * inch);
        BackleftMotor.setTargetPosition(-distance * inch);
        BackrightMotor.setTargetPosition(distance * inch);

        FrontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontleftMotor.setPower(power);
        FrontrightMotor.setPower(power);
        BackleftMotor.setPower(power);
        BackrightMotor.setPower(power);

        //Forward
        while (opModeIsActive() && (FrontleftMotor.isBusy() || FrontrightMotor.isBusy() || BackleftMotor.isBusy() || BackrightMotor.isBusy())) {

        }
        FrontleftMotor.setPower(0);
        FrontrightMotor.setPower(0);
        BackleftMotor.setPower(0);
        BackrightMotor.setPower(0);

    }

    private void moveFoward (int distance, double power) {

        //int power = 0;
        FrontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FrontleftMotor.setTargetPosition(-distance * inch); // check movements of the motors for negative and positive values
        FrontrightMotor.setTargetPosition(-distance * inch);
        BackleftMotor.setTargetPosition(-distance * inch);
        BackrightMotor.setTargetPosition(-distance * inch);

        FrontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontleftMotor.setPower(power);
        FrontrightMotor.setPower(power);
        BackleftMotor.setPower(power);
        BackrightMotor.setPower(power);

        //Forward
        while (opModeIsActive() && (FrontleftMotor.isBusy() || FrontrightMotor.isBusy() || BackleftMotor.isBusy() || BackrightMotor.isBusy())) {

        }
        FrontleftMotor.setPower(0);
        FrontrightMotor.setPower(0);
        BackleftMotor.setPower(0);
        BackrightMotor.setPower(0);

    }

}