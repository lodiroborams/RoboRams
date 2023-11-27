package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Blue extends LinearOpMode {

    private DcMotor Frontleftmotor;
    private double motorFLZeroPower = 0.0;
    private double motorFLPower = 1.0;
    private int motorFLPositionOne = 0;
    private int motorFLPositionTwo = 1000;

    private DcMotor Frontrightmotor;
    private double motorFRZeroPower = 0.0;
    private double motorFRPower = 1.0;
    private int motorFRPositionOne = 0;
    private int motorFRPositionTwo = 1000;

    private DcMotor Backleftmotor;
    private double motorBLZeroPower = 0.0;
    private double motorBLPower = 1.0;
    private int motorBLPositionOne = 0;
    private int motorBLPositionTwo = 1000;

    private DcMotor Backrightmotor;
    private double motorBRZeroPower = 0.0;
    private double motorBRPower = 1.0;
    private int motorBRPositionOne = 0;
    private int motorBRPositionTwo = 1000;



    @Override
    public void runOpMode() {
        // Initialize motors
        Frontleftmotor = hardwareMap.get(DcMotor.class, "front_left");
        Frontrightmotor = hardwareMap.get(DcMotor.class, "front_right");
        Backleftmotor = hardwareMap.get(DcMotor.class, "back_left");
        Backrightmotor = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions
        Frontleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Frontrightmotor.setDirection(DcMotor.Direction.FORWARD);
        Backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Backrightmotor.setDirection(DcMotor.Direction.FORWARD);

        Frontleftmotor.setPower(motorFLPower);
        Backleftmotor.setPower(motorBLPower);
        Frontrightmotor.setPower(motorFRPower);
        Backrightmotor.setPower(motorBRPower);

        Frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {







            }


        }


    }

    private void moveFoward (int position) {
        Frontleftmotor.setTargetPosition(position);
        Frontrightmotor.setTargetPosition(position);
        Backleftmotor.setTargetPosition(position);
        Backrightmotor.setTargetPosition(position);

        Frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Frontleftmotor.setPower(motorFLPower);
        Frontrightmotor.setPower(motorFRPower);
        Backleftmotor.setPower(motorBLPower);
        Backrightmotor.setPower(motorBRPower);

        while (opModeIsActive() && (Frontleftmotor.isBusy() || Frontrightmotor.isBusy() || Backleftmotor.isBusy() || Backrightmotor.isBusy())) {
        }
        Frontleftmotor.setPower(motorFLZeroPower);
        Frontrightmotor.setPower(motorFRPower);
        Backleftmotor.setPower(motorBLZeroPower);
        Backrightmotor.setPower(motorBRPower);

    }

}