package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Drive extends LinearOpMode {
    private DcMotor FrontleftMotor;
    private DcMotor FrontrightMotor;
    private DcMotor BackleftMotor;
    private DcMotor BackrightMotor;

    private DcMotorEx ArmMotor;
    private double ArmMotorZeroPower = 0.0;
    private double ArmMotorMaxPower = 1.0;
    private double currentVelocity = 0.0;
    private double maxVelocity = 0.0;

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
        ArmMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        Claw = hardwareMap.get(Servo.class, "claw");
        Wrist = hardwareMap.get(Servo.class, "wrist");
        Launcher = hardwareMap.get(Servo.class, "launcher");
        Unstuck1 = hardwareMap.get(Servo.class, "unstuck1");
        Unstuck2 = hardwareMap.get(Servo.class, "unstuck2");



        FrontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //PID for Arm motor
        ArmMotor.setDirection(DcMotorEx.Direction.FORWARD);
        ArmMotor.setPower(ArmMotorZeroPower);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();
        if (opModeIsActive()) {

            Wrist.setDirection(Servo.Direction.REVERSE);
            Wrist.setPosition(wristInitPosition);

            Claw.setPosition(.1);

            Wrist.setPosition(.4);

            Launcher.setPosition(0);

            while (opModeIsActive()) {

                ArmMotorMaxVelocityTest();
                motorTelemetry();



                double strafe_stick = gamepad1.right_stick_x;     // TODO ITERATION 1 gamepad 1
                double rotate_stick = -gamepad1.left_stick_x; // counteract improer strafing
                double forwardback_stick = gamepad1.right_stick_y;

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
                    Claw.setPosition(.9);

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
    public void ArmMotorMaxVelocityTest(){
        ArmMotor.setPower(ArmMotorMaxPower);
        currentVelocity = ArmMotor.getVelocity();
        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }
    }

    public void motorTelemetry() {
        telemetry.addData("Current Power", ArmMotor.getPower());
        telemetry.addData("Maximum Velocity", maxVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();
    }
}
