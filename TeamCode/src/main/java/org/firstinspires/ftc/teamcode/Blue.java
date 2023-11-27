package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Blue extends LinearOpMode {

    private DcMotor Frontleftmotor;
    private DcMotor Frontrightmotor;
    private DcMotor Backleftmotor;
    private DcMotor Backrightmotor;



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

        Frontleftmotor.setPower(0);
        Backleftmotor.setPower(0);
        Frontrightmotor.setPower(0);
        Backrightmotor.setPower(0);

        Frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
    }

}