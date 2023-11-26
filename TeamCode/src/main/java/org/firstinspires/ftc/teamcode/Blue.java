package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class Blue extends LinearOpMode {

    private DcMotor Frontleftmotor;
    private DcMotor Frontrightmotor;
    private DcMotor Backleftmotor;
    private DcMotor Backrightmotor;

    private double COUNTS_PER_REVOLUTION = 537.6;
    private double WHEEL_DIAMETER = 4.0; // Inches
    private double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER * Math.PI);
    private double MOVE_POWER = 0.5; // Adjust power as needed
    private double MOVE_DISTANCE = 4.0; // Inches
    private int MOVE_TIMEOUT = 2000; // 2 seconds

    @Override
    public void runOpMode() {
        // Initialize motors
        Frontleftmotor = hardwareMap.get(DcMotor.class, "front_left");
        Frontrightmotor = hardwareMap.get(DcMotor.class, "front_right");
        Backleftmotor = hardwareMap.get(DcMotor.class, "back_left");
        Backrightmotor = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions
        Frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        Frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
        Backleftmotor.setDirection(DcMotor.Direction.REVERSE);
        Backrightmotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        // Move the robot forward for 4 inches
        moveForward(MOVE_DISTANCE);

        // Stop the robot
        stopRobot();
    }

    private void moveForward(double inches) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        // Set target position for all motors
        Frontleftmotor.setTargetPosition(targetPosition);
        Frontrightmotor.setTargetPosition(targetPosition);
        Backleftmotor.setTargetPosition(targetPosition);
        Backrightmotor.setTargetPosition(targetPosition);

        // Set motor modes to RUN_TO_POSITION
        Frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        Frontleftmotor.setPower(MOVE_POWER);
        Frontrightmotor.setPower(MOVE_POWER);
        Backleftmotor.setPower(MOVE_POWER);
        Backrightmotor.setPower(MOVE_POWER);

        // Set a timeout for the movement
        long startTime = System.currentTimeMillis();

        // Wait until all motors reach the target position or timeout
        while (opModeIsActive() &&
                (Frontleftmotor.isBusy() ||
                        Frontrightmotor.isBusy() ||
                        Backleftmotor.isBusy() ||
                        Backrightmotor.isBusy()) &&
                (System.currentTimeMillis() - startTime < MOVE_TIMEOUT)) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Frontleftmotor Position", Frontleftmotor.getCurrentPosition());
            telemetry.update();
        }

        // Set motor power back to 0
        Frontleftmotor.setPower(0);
        Frontrightmotor.setPower(0);
        Backleftmotor.setPower(0);
        Backrightmotor.setPower(0);

        // Set motor modes back to RUN_USING_ENCODER
        Frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopRobot() {
        // Set motor power to 0 for all motors
        Frontleftmotor.setPower(0);
        Frontrightmotor.setPower(0);
        Backleftmotor.setPower(0);
        Backrightmotor.setPower(0);
    }
}