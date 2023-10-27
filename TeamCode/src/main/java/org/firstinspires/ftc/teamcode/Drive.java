package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Drive extends LinearOpMode {
    private DcMotor FrontleftMotor;
    private DcMotor FrontrightMotor;
    private DcMotor BackleftMotor;
    private DcMotor BackrightMotor;


    //private Servo positioning;

    private Servo lift1;
    private Servo lift2;


    public void runOpMode() {
        FrontleftMotor = hardwareMap.get(DcMotor.class, "front_left");
        FrontrightMotor = hardwareMap.get(DcMotor.class, "front_right");
        BackleftMotor = hardwareMap.get(DcMotor.class, "back_left");
        BackrightMotor = hardwareMap.get(DcMotor.class, "back_right");
        lift1 = hardwareMap.get(Servo.class, "left_servo");
        lift2 = hardwareMap.get(Servo.class, "right_servo");


        FrontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            
            double lift1.setPosition(0);
            double lift2.setPosition(0);
            
            while (opModeIsActive()) {


                double y = -gamepad1.left_stick_x;     // TODO ITERATION 1 gamepad 1
                double x = gamepad1.left_stick_y; // counteract improer strafing
                double rx = gamepad1.right_stick_x;


                double rotX = x * Math.cos(0) - y * Math.sin(0);
                double rotY = x * Math.sin(0) + y * Math.cos(0);


                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontleftPower = (rotY + rotX - rx) / denominator;
                double backleftPower = (rotY - rotX + rx) / denominator;
                double frontrightPower = (rotY - rotX - rx) / denominator;
                double backrightPower = (rotY + rotX + rx) / denominator;

                FrontleftMotor.setPower(frontleftPower);
                FrontrightMotor.setPower(frontrightPower);
                BackleftMotor.setPower(backleftPower);
                BackrightMotor.setPower(backrightPower);


                if (gamepad1.a) {
                    ;

                    lift1.setPosition(.4);
                    lift2.setPosition(.4);
                }

                if (gamepad1.x) {
                    ;

                    lift1.setPosition(.6);
                    lift2.setPosition(.6);
                }
            }
        }
    }
}
