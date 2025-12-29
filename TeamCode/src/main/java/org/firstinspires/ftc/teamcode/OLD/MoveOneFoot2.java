package org.firstinspires.ftc.teamcode.OLD;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class MoveOneFoot2 extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        for (int i = 0; i < 4; i++) {
            moveForward(24);  // Move forward 24 inches (2 feet)
            stopRobot();      // Stop the robot
            turn(90);         // Turn 90 degrees
        }
    }

    private void moveForward(double inches) {
        // Set power to the motors to move forward
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);

        // Convert inches to encoder counts
        final double COUNTS_PER_INCH = 89.06;   // Example value
        int targetPosition = (int) (inches * COUNTS_PER_INCH); // Define COUNTS_PER_INCH
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        // Start moving
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until the motors reach the target position
        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // Optionally add telemetry here
        }

        stopRobot(); // Ensure the robot stops
    }

    private void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void turn(int degrees) {
        // Simple turning logic
        leftMotor.setPower(1.0);
        rightMotor.setPower(-1.0);

        // Convert degrees to encoder counts
        double COUNTS_PER_DEGREE = 3.11;   // Example value
        int targetPosition = (int) (degrees * COUNTS_PER_DEGREE); // Define COUNTS_PER_DEGREE

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(-targetPosition);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // Optionally add telemetry here
        }

        stopRobot(); // Ensure the robot stops
    }
}

