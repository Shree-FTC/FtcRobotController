package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@Autonomous(name="SimpleAuto", group="Autonomous")
public class SimpleAuto extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;

    private CRServo speedServo1 = null;
    private CRServo torqueServo1 = null;
    private CRServo speedServo2 = null;
    private CRServo torqueServo2 = null;

    private HuskyLens huskyLens = null;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 0.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.09449;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.5;

    //  SET TARGET DISTANCE
    private static final double locx = 0.0;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        speedServo1 = hardwareMap.get(CRServo.class, "speedServo1");
        torqueServo1 = hardwareMap.get(CRServo.class, "torqueServo1");
        speedServo2 = hardwareMap.get(CRServo.class, "speedServo2");
        torqueServo2 = hardwareMap.get(CRServo.class, "torqueServo2");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveForward(48, DRIVE_SPEED);
            sleep(500);

            turnRight(90, TURN_SPEED);
            sleep(500);

            double distanceFromTag = readAprilTagDistance();

            if (distanceFromTag > 0) {
                double moveDistance = distanceFromTag - locx;

                if (Math.abs(moveDistance) > 1.0) {
                    if (moveDistance > 0) {
                        driveForward(moveDistance, DRIVE_SPEED);
                    } else {
                        driveBackward(Math.abs(moveDistance), DRIVE_SPEED);
                    }
                    sleep(500);
                }
            }

            activateServos();
            sleep(3000);
            stopServos();
        }
    }

    private void driveForward(double inches, double speed) {
        int targetPosition = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetPosition);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetPosition);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPosition);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {
        }

        stopMotors();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveBackward(double inches, double speed) {
        driveForward(-inches, speed);
    }

    private void turnRight(double degrees, double speed) {
        double inches = degrees * 0.2;
        int targetPosition = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetPosition);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - targetPosition);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPosition);
        backRight.setTargetPosition(backRight.getCurrentPosition() - targetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {
        }

        stopMotors();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double readAprilTagDistance() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks.length > 0) {
            HuskyLens.Block block = blocks[0];
            int blockWidth = block.width;
            double estimatedDistance = (100.0 / blockWidth) * 10.0;
            return estimatedDistance;
        }

        return -1;
    }

    private void activateServos() {
        // Pair 1: clockwise at 80%
        speedServo1.setPower(0.8);
        torqueServo1.setPower(0.8);

        // Pair 2: counter-clockwise at 80%
        speedServo2.setPower(-0.8);
        torqueServo2.setPower(-0.8);
    }

    private void stopServos() {
        speedServo1.setPower(0);
        torqueServo1.setPower(0);
        speedServo2.setPower(0);
        torqueServo2.setPower(0);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}