package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="nanoteleop",group = "TeleOp")
public class drivetrainteleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    protected DcMotor frontleft;
    protected DcMotor frontright;
    protected DcMotor backleft;
    protected DcMotor backright;


    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");


        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double drive = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_y;
            //turn = turn / 1.5;
            double leftPower = Range.clip(drive + turn, -0.5, 0.5);
            double rightPower = Range.clip(drive - turn, -0.5, 0.5);

            frontleft.setPower(leftPower);
            frontright.setPower(rightPower);
            backleft.setPower(leftPower);
            backright.setPower(rightPower);
            // Sliding arm control using D-pad


            if (gamepad1.dpad_left) {
                strafeLeft();
            } else if (gamepad1.dpad_right) {
                strafeRight();
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f), slide (%.2f)", leftPower, rightPower);
            telemetry.update();

        }

    }

    public void moveforward() {
        frontleft.setPower(1);
        frontright.setPower(1);
        backleft.setPower(1);
        backright.setPower(1);
    }

    public void turnright() {
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(1);
        backright.setPower(-1);
    }

    public void turnleft() {
        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(-1);
        backright.setPower(1);
    }

    public void movebackward() {
        frontleft.setPower(-1);
        frontright.setPower(-1);
        backleft.setPower(-1);
        backright.setPower(-1);
    }

    public void Stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }


    public void strafeLeft() {
        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(1);
        backright.setPower(-1);

    }

    public void strafeRight() {
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(-1);
        backright.setPower(1);

    }
}