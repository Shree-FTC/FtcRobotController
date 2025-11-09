package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="SimpleTele", group="TeleOp")
public class SimpleTele extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;

    private CRServo speedServo1 = null;
    private CRServo torqueServo1 = null;
    private CRServo speedServo2 = null;
    private CRServo torqueServo2 = null;
    private boolean shooterOn = false;
    private boolean bPressedLastLoop = false;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        speedServo1 = hardwareMap.get(CRServo.class, "speedServo1");
        torqueServo1 = hardwareMap.get(CRServo.class, "torqueServo1");
        speedServo2 = hardwareMap.get(CRServo.class, "speedServo2");
        torqueServo2 = hardwareMap.get(CRServo.class, "torqueServo2");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;

            double rotate = gamepad2.right_stick_x;

            double flPower = drive + strafe + rotate;
            double frPower = drive - strafe - rotate;
            double blPower = drive - strafe + rotate;
            double brPower = drive + strafe - rotate;

            double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));

            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            speedServo1.setPower(-1);
            torqueServo1.setPower(-1);
            speedServo2.setPower(1);
            torqueServo2.setPower(1);

            boolean bPressedNow = gamepad2.b;
            if (bPressedNow && !bPressedLastLoop) {
                shooterOn = !shooterOn;
            }
            bPressedLastLoop = bPressedNow;

            if (shooterOn) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }

        }
    }
}