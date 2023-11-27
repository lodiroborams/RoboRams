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
    double wristInitPosition = .50;
    double wristSensitivity = .50;
    private Servo Launcher;
    private Servo Wrist;
    private Servo Unstuck1;
    private Servo Unstuck2;



    public void runOpMode() {
        FrontleftMotor = hardwareMap.get(DcMotor.class, "front_left");
        FrontrightMotor = hardwareMap.get(DcMotor.class, "front_right");
        BackleftMotor = hardwareMap.get(DcMotor.class, "back_left");
        BackrightMotor = hardwareMap.get(DcMotor.class, "back_right");
        ArmMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        Claw = hardwareMap.get(Servo.class, "claw");
        Wrist = hardwareMap.get(Servo.class, "wrist");
        Launcher = hardwareMap.get(Servo.class, "launcher");
        Unstuck1 = hardwareMap.get(Servo.class, "unstuck1");
        Unstuck2 = hardwareMap.get(Servo.class, "unstuck2");



        FrontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {

            ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            Wrist.setDirection(Servo.Direction.REVERSE);
            Wrist.setPosition(wristInitPosition);

            Claw.setPosition(.1);

            Wrist.setPosition(.4);

            Launcher.setPosition(0);

            while (opModeIsActive()) {
/*
  see what the inputs are for this code right now right down what you think
  left stick should be rotating controls
  right stick should be forward, left, back, right
*/
                double strafe_stick = -gamepad1.left_stick_y;     // TODO ITERATION 1 gamepad 1
                double rotate_stick = gamepad1.left_stick_x; // counteract improer strafing
                double forwardback_stick = -gamepad1.right_stick_y;

                double updown_stick = -gamepad2.left_stick_y;     // TODO ITERATION 1

                double frontleftPower = forwardback_stick + rotate_stick - strafe_stick;
                double frontrightPower = forwardback_stick - rotate_stick + strafe_stick;
                double backleftPower = forwardback_stick + rotate_stick + strafe_stick;
                double backrightPower = forwardback_stick -rotate_stick - strafe_stick;

                double updown_stickPower = updown_stick;


                Wrist.setPosition(wristInitPosition + (gamepad2.right_stick_y * wristSensitivity));


                FrontleftMotor.setPower(frontleftPower);
                FrontrightMotor.setPower(frontrightPower);
                BackleftMotor.setPower(backleftPower);
                BackrightMotor.setPower(backrightPower);

                ArmMotor.setPower(updown_stickPower);



                if (gamepad2.right_bumper) {
                    Claw.setPosition(.1);

                }


                if (gamepad2.left_bumper) {
                    Claw.setPosition(.2);

                }

                if (gamepad2.x){
                    Launcher.setPosition(1);
                    
                }

                if (gamepad2.a){
                    Unstuck1.setPosition(1);
                    Unstuck2.setPosition(1);
                }
                else {
                Unstuck1.setPosition(0);
                Unstuck2.setPosition(0);
                
                }

            }
        }
    }
}
