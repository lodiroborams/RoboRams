package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class RedParking1 extends LinearOpMode {

    //Motor
    private DcMotor FrontleftMotor;
    private DcMotor FrontrightMotor;
    private DcMotor BackleftMotor;
    private DcMotor BackrightMotor;

    int tics = 1440;
    int revolution = 16;
    int inch = tics / revolution;

    public void runOpMode() throws InterruptedException {
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


        waitForStart();
        if (opModeIsActive()) {

            strafeleft(55,1);


        }

    }


    private void move(int distance, double power) {

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

}
