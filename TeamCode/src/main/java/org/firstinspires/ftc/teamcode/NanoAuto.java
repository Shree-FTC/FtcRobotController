package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@Autonomous(name="NanoAuto", group="Auto")
public class NanoAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;
    private DcMotor shooter;
    private Servo sortWheel, kicker;

    private Servo turretYaw;
    private DcMotor turretPitch;

    private Limelight3A limelight;

    private static final double COUNTS_PER_INCH = 50;
    private static final double SHOOTER_POWER = 0.9;
    private static final double DRIVE_POWER = 0.5;
    static final double TRACK_WIDTH_INCHES = 14.0;
    static final double TURN_POWER = 0.3;

    private static final double TURRET_PITCH_SPEED = 0.3;
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.5828;
    private static final double YAW_MAX = 0.25;


    private double yawPosition = YAW_CENTER;
    private double[] spindexSteps = {0.0, 0.1667, 0.3333, 0.5, 0.6667, 0.8333};

    private PIDController yawPID = new PIDController(0.015, 0.0001, 0.005);
    private PIDController pitchPID = new PIDController(0.02, 0.0001, 0.008);

    private class PIDController {
        double kP, kI, kD;
        double integral = 0, prevError = 0;

        PIDController(double p, double i, double d) {
            kP = p; kI = i; kD = d;
        }

        double calculate(double error) {
            integral += error;
            double derivative = error - prevError;
            prevError = error;

            return (kP * error) + (kI * integral) + (kD * derivative);
        }

        void reset() {
            integral = 0;
            prevError = 0;
        }
    }

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");
        shooter = hardwareMap.dcMotor.get("shooter");
        sortWheel = hardwareMap.servo.get("sortWheel");
        kicker = hardwareMap.servo.get("kicker");

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);

        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretYaw.setPosition(YAW_CENTER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker.setPosition(0.0);
        sortWheel.setPosition(spindexSteps[0]);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //First 3
            driveForward(-48);
            calculateShooterPower(48);
            autoAimAndShoot();
            //Second 3
            turnDegrees(45);
            Intake();
            driveForward(-52);
            turnDegrees(-45);
            calculateShooterPower(48);
            autoAimAndShoot();
            //Third 3
            turnDegrees(45);
            driveSideways(48);
            Intake();
            driveForward(-52);
            driveSideways(-48);
            turnDegrees(-45);
            calculateShooterPower(48);
            autoAimAndShoot();


        }
    }

    private void autoAimAndShoot() {
        long startTime = System.currentTimeMillis();
        boolean targetAcquired = false;

        yawPID.reset();
        pitchPID.reset();

        telemetry.addLine("Acquiring target...");
        telemetry.update();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 3000) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    targetAcquired = true;

                    LLResultTypes.FiducialResult target = fiducials.get(0);
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f.getFiducialId() == 20 || f.getFiducialId() == 24) {
                            target = f;
                            break;
                        }
                    }

                    double tx = target.getTargetXDegrees();
                    double ty = target.getTargetYDegrees();
                    double ta = target.getTargetArea();
                    double estimatedDistance = 0.55 / ta;

                    double yawCorrection = yawPID.calculate(tx);
                    yawPosition += yawCorrection;
                    yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
                    turretYaw.setPosition(yawPosition);

                    double pitchPower = pitchPID.calculate(-ty);
                    pitchPower = Math.max(-TURRET_PITCH_SPEED, Math.min(TURRET_PITCH_SPEED, pitchPower));
                    turretPitch.setPower(pitchPower);

                    telemetry.addData("Target ID", target.getFiducialId());
                    telemetry.addData("TX", tx);
                    telemetry.addData("TY", ty);
                    telemetry.addData("Distance", estimatedDistance);
                    telemetry.update();

                    if (Math.abs(tx) < 1.5 && Math.abs(ty) < 1.5) {
                        break;
                    }
                }
            }

            sleep(50);
        }

        turretPitch.setPower(0);

        if (!targetAcquired) {
            telemetry.addLine("WARNING: No target found!");
            telemetry.update();
            sleep(1000);
        }

        telemetry.addLine("Starting shooter...");
        telemetry.update();

        LLResult finalResult = limelight.getLatestResult();
        double shooterPower = SHOOTER_POWER;

        if (finalResult != null && finalResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = finalResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                double ta = fiducials.get(0).getTargetArea();
                double distance = 0.55 / ta;
                shooterPower = calculateShooterPower(distance);
            }
        }

        shooter.setPower(shooterPower);
        sleep(1000);

        int[] shootingIndices = {3, 4, 5};

        for (int i = 0; i < 3; i++) {
            telemetry.addData("Shooting ball", i + 1);
            telemetry.update();

            sortWheel.setPosition(spindexSteps[shootingIndices[i]]);
            sleep(500);

            kicker.setPosition(1.0);
            sleep(1000);
            kicker.setPosition(0.0);
            sleep(700);
        }

        shooter.setPower(0.0);
        telemetry.addLine("Shooting complete!");
        telemetry.update();
    }

    private void rotateSpindexDegrees(double degrees) {
        double rotations = degrees / 360.0;
        double targetPosition = rotations;

        targetPosition = targetPosition % 1.0;
        if (targetPosition < 0) targetPosition += 1.0;

        sortWheel.setPosition(targetPosition);
    }

    private double calculateShooterPower(double distanceInches) {
        double minPower = 0.57;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 140;

        distanceInches = Math.max(minDist, Math.min(maxDist, distanceInches));

        return minPower +
                (distanceInches - minDist) *
                        (maxPower - minPower) /
                        (maxDist - minDist);
    }

    private void driveForward(double inches) {
        int targetCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(targetCounts);
        frontRight.setTargetPosition(targetCounts);
        backLeft.setTargetPosition(targetCounts);
        backRight.setTargetPosition(targetCounts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);

        while (opModeIsActive() &&
                frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnDegrees(double degrees) {
        double turnCircumference = Math.PI * TRACK_WIDTH_INCHES;
        double distance = (degrees / 360.0) * turnCircumference;

        int counts = (int)(distance * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(counts);
        backLeft.setTargetPosition(counts);
        frontRight.setTargetPosition(-counts);
        backRight.setTargetPosition(-counts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(TURN_POWER);
        frontRight.setPower(TURN_POWER);
        backLeft.setPower(TURN_POWER);
        backRight.setPower(TURN_POWER);

        while (opModeIsActive() &&
                frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void driveSideways(double inches) {
        int targetCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(targetCounts);
        frontRight.setTargetPosition(-targetCounts);
        backLeft.setTargetPosition(-targetCounts);
        backRight.setTargetPosition(targetCounts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);

        while (opModeIsActive() &&
                frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void Intake(){
        rotateSpindexDegrees(60);
        rollerMotor.setPower(1);
        driveForward(24);
        rotateSpindexDegrees(120);
        sleep(750);
        driveForward(2);
        sleep(750);
        driveForward(2);
        sleep(500);
        rollerMotor.setPower(0);
        rotateSpindexDegrees(60);
    }
}