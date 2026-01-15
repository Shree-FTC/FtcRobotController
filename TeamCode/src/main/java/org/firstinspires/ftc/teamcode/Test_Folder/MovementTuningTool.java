package org.firstinspires.ftc.teamcode.Test_Folder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode to tune INCHES_PER_SECOND and DEGREES_PER_SECOND constants
 *
 * Instructions:
 * 1. Run LINEAR test: Robot drives forward for set time at full power
 *    - Measure actual distance traveled
 *    - Enter measurement when prompted
 * 2. Run ROTATION test: Robot rotates for set time at full power
 *    - Measure actual angle rotated (use gyro or visual markers)
 *    - Enter measurement when prompted
 * 3. Repeat tests 3-5 times for accuracy
 * 4. Constants will be calculated and displayed
 */
@TeleOp(name="Movement Tuning Tool", group="Tuning")
public class MovementTuningTool extends LinearOpMode {

    // Hardware
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    // Test parameters
    private static final double TEST_POWER = 1.0;
    private static final double LINEAR_TEST_TIME = 3.0; // seconds
    private static final double ROTATION_TEST_TIME = 2.0; // seconds

    // Results storage
    private double[] linearResults = new double[10];
    private double[] rotationResults = new double[10];
    private int linearCount = 0;
    private int rotationCount = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust for your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Movement Tuning Tool");
        telemetry.addLine("-------------------");
        telemetry.addLine("DPAD UP: Linear Test");
        telemetry.addLine("DPAD DOWN: Rotation Test");
        telemetry.addLine("X: Clear Results");
        telemetry.addLine();
        telemetry.addLine("Ready to start tuning");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Linear movement test
            if (gamepad1.dpad_up) {
                runLinearTest();
                sleep(500); // Debounce
            }

            // Rotation test
            if (gamepad1.dpad_down) {
                runRotationTest();
                sleep(500); // Debounce
            }

            // Clear results
            if (gamepad1.x) {
                linearCount = 0;
                rotationCount = 0;
                telemetry.addLine("Results cleared!");
                telemetry.update();
                sleep(500);
            }

            displayResults();
        }
    }

    private void runLinearTest() {
        telemetry.addLine("LINEAR TEST STARTING");
        telemetry.addData("Duration", "%.1f seconds", LINEAR_TEST_TIME);
        telemetry.addLine("Robot will drive forward...");
        telemetry.update();
        sleep(2000);

        // Drive forward at full power
        setDrivePower(TEST_POWER, TEST_POWER, TEST_POWER, TEST_POWER);

        timer.reset();
        while (timer.seconds() < LINEAR_TEST_TIME && opModeIsActive()) {
            telemetry.addLine("DRIVING...");
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), LINEAR_TEST_TIME);
            telemetry.update();
        }

        stopDrive();

        // Get measurement from user via gamepad
        telemetry.clear();
        telemetry.addLine("Test Complete!");
        telemetry.addLine("Measure the distance traveled in INCHES");
        telemetry.addLine();
        telemetry.addLine("Use DPAD LEFT/RIGHT to adjust by 1 inch");
        telemetry.addLine("Use BUMPERS to adjust by 0.1 inch");
        telemetry.addLine("Press A to confirm");
        telemetry.update();

        double measurement = 24.0; // Default starting value

        while (!gamepad1.a && opModeIsActive()) {
            if (gamepad1.dpad_right) {
                measurement += 1.0;
                sleep(150);
            }
            if (gamepad1.dpad_left) {
                measurement -= 1.0;
                sleep(150);
            }
            if (gamepad1.right_bumper) {
                measurement += 0.1;
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                measurement -= 0.1;
                sleep(100);
            }

            telemetry.clear();
            telemetry.addLine("Enter measured distance:");
            telemetry.addData("Distance", "%.1f inches", measurement);
            telemetry.addLine();
            telemetry.addLine("DPAD L/R: ±1 inch");
            telemetry.addLine("BUMPERS: ±0.1 inch");
            telemetry.addLine("Press A to confirm");
            telemetry.update();
        }

        // Calculate and store result
        if (linearCount < linearResults.length) {
            linearResults[linearCount] = measurement / LINEAR_TEST_TIME;
            linearCount++;
        }

        sleep(500); // Debounce
    }

    private void runRotationTest() {
        telemetry.addLine("ROTATION TEST STARTING");
        telemetry.addData("Duration", "%.1f seconds", ROTATION_TEST_TIME);
        telemetry.addLine("Robot will rotate clockwise...");
        telemetry.update();
        sleep(2000);

        // Rotate clockwise at full power
        setDrivePower(TEST_POWER, TEST_POWER, -TEST_POWER, -TEST_POWER);

        timer.reset();
        while (timer.seconds() < ROTATION_TEST_TIME && opModeIsActive()) {
            telemetry.addLine("ROTATING...");
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), ROTATION_TEST_TIME);
            telemetry.update();
        }

        stopDrive();

        // Get measurement from user via gamepad
        telemetry.clear();
        telemetry.addLine("Test Complete!");
        telemetry.addLine("Measure the angle rotated in DEGREES");
        telemetry.addLine();
        telemetry.addLine("Use DPAD LEFT/RIGHT to adjust by 10 degrees");
        telemetry.addLine("Use BUMPERS to adjust by 1 degree");
        telemetry.addLine("Press A to confirm");
        telemetry.update();

        double measurement = 180.0; // Default starting value

        while (!gamepad1.a && opModeIsActive()) {
            if (gamepad1.dpad_right) {
                measurement += 10.0;
                sleep(150);
            }
            if (gamepad1.dpad_left) {
                measurement -= 10.0;
                sleep(150);
            }
            if (gamepad1.right_bumper) {
                measurement += 1.0;
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                measurement -= 1.0;
                sleep(100);
            }

            telemetry.clear();
            telemetry.addLine("Enter measured angle:");
            telemetry.addData("Angle", "%.0f degrees", measurement);
            telemetry.addLine();
            telemetry.addLine("DPAD L/R: ±10 degrees");
            telemetry.addLine("BUMPERS: ±1 degree");
            telemetry.addLine("Press A to confirm");
            telemetry.update();
        }

        // Calculate and store result
        if (rotationCount < rotationResults.length) {
            rotationResults[rotationCount] = measurement / ROTATION_TEST_TIME;
            rotationCount++;
        }

        sleep(500); // Debounce
    }

    private void displayResults() {
        telemetry.clear();
        telemetry.addLine("=== TUNING RESULTS ===");
        telemetry.addLine();

        // Linear results
        telemetry.addLine("LINEAR MOVEMENT:");
        telemetry.addData("Tests Run", linearCount);
        if (linearCount > 0) {
            double avg = calculateAverage(linearResults, linearCount);
            telemetry.addData("Average Speed", "%.2f in/sec", avg);
            telemetry.addLine();
            telemetry.addLine("INCHES_PER_SECOND = " + String.format("%.2f", avg) + ";");
        }

        telemetry.addLine();

        // Rotation results
        telemetry.addLine("ROTATION:");
        telemetry.addData("Tests Run", rotationCount);
        if (rotationCount > 0) {
            double avg = calculateAverage(rotationResults, rotationCount);
            telemetry.addData("Average Speed", "%.2f deg/sec", avg);
            telemetry.addLine();
            telemetry.addLine("DEGREES_PER_SECOND = " + String.format("%.2f", avg) + ";");
        }

        telemetry.addLine();
        telemetry.addLine("-------------------");
        telemetry.addLine("DPAD UP: Linear Test");
        telemetry.addLine("DPAD DOWN: Rotation Test");
        telemetry.addLine("X: Clear Results");
        telemetry.update();
    }

    private double calculateAverage(double[] values, int count) {
        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += values[i];
        }
        return sum / count;
    }

    private void setDrivePower(double lf, double lr, double rf, double rr) {
        frontLeft.setPower(lf);
        backLeft.setPower(lr);
        frontRight.setPower(rf);
        backRight.setPower(rr);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }
}