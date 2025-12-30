package org.firstinspires.ftc.teamcode.Test_Folder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Continuous Motor Test", group="Tests")
public class Conmoter extends LinearOpMode {

    private DcMotor motor;


    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                motor.setPower(-1.0);
            }
        }

        motor.setPower(0);
    }
}

