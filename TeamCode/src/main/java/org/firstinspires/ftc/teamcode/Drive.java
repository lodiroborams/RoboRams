package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Drive extends LinearOpMode {
    private DcMotor FrontleftMotor;
    private DcMotor FrontrightMotor;
    private DcMotor BackleftMotor;
    private DcMotor BackrightMotor;
    private DcMotor ArmMotor;

    //Servo
    private Servo Claw;
    private Servo Launcher;
    private Servo Wrist;


    public void runOpMode() {
        FrontleftMotor = hardwareMap.get(DcMotor.class, "front_left");
        FrontrightMotor = hardwareMap.get(DcMotor.class, "front_right");
        BackleftMotor = hardwareMap.get(DcMotor.class, "back_left");
        BackrightMotor = hardwareMap.get(DcMotor.class, "back_right");
        ArmMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        Claw = hardwareMap.get(Servo.class, "claw");
        Wrist = hardwareMap.get(Servo.class, "wrist");
        Launcher = hardwareMap.get(Servo.class, "launcher");


        FrontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {

            ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            Claw.setPosition(0);

            Wrist.setPosition(0);

            Launcher.setPosition(0);

            while (opModeIsActive()) {
/*
  see what the inputs are for this code right now right down what you think
  left stick should be rotating controls
  right stick should be forward, left, back, right
*/
                double strafe_stick = -gamepad1.left_stick_y;     // TODO ITERATION 1 gamepad 1
                double rotate_stick = -gamepad1.left_stick_x; // counteract improer strafing
                double forwardback_stick = -gamepad1.right_stick_y;

                double updown_stick = -gamepad2.left_stick_y;     // TODO ITERATION 1
                double wrist_updown = -gamepad2.right_stick_y;

                double frontleftPower = forwardback_stick + rotate_stick - strafe_stick;
                double frontrightPower = forwardback_stick - rotate_stick + strafe_stick;
                double backleftPower = forwardback_stick + rotate_stick + strafe_stick;
                double backrightPower = forwardback_stick -rotate_stick - strafe_stick;

                double updown_stickPower = updown_stick;
                double wrist_updownPower = wrist_updown;


                FrontleftMotor.setPower(-frontleftPower);
                FrontrightMotor.setPower(-frontrightPower);
                BackleftMotor.setPower(backleftPower);
                BackrightMotor.setPower(backrightPower);

                ArmMotor.setPower(updown_stickPower);

                Wrist.setPosition(updown_stickPower);




                if (gamepad2.right_bumper) {
                    Claw.setPosition(0);

                }


                if (gamepad2.left_bumper) {
                    Claw.setPosition(.1);

                }

                if (gamepad2.x){
                    Launcher.setPosition(1);
                }

            }
        }
    }
}