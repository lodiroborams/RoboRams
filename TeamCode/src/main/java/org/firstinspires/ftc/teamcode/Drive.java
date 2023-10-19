package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive")
public class Drive extends OpMode {

    DcMotor motor;

    @Override
    public void init() {

        telemetry.addData("Initialization:", "is a success");
                telemetry.update();

    }

    @Override
    public void loop() {

    }
}
