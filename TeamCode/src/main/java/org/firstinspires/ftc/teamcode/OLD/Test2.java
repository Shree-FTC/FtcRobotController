package org.firstinspires.ftc.teamcode.OLD;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Test2 extends LinearOpMode {


    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {


        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        moveForward(10);  // Move 10 feet

        // Turn 90 degrees (adjust this for your robot's turn behavior)
        turnRight(90);  // Turn 90 degrees to the right

        // Move 2 feet forward
        moveForward(2);  // Move 2 feet
    }

    // Method to move the robot forward a specific distance in feet
    private void moveForward(double feet) {
        int ticks = (int) (feet * 537);  // Calculate encoder ticks for the given feet (adjust 500 as needed)

        // Reset encoders before starting the move
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run to position
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target position
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        // Set power to motors to start moving
        leftMotor.setPower(0.5);  // 50% power
        rightMotor.setPower(0.5);  // 50% power

        // Wait until the motors reach the target position
        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            telemetry.addData("Moving", "Distance: %d feet", feet);
            telemetry.update();
            idle();
        }

        // Stop the motors after reaching the target position
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // Method to turn the robot to the right by 90 degrees
    private void turnRight(int degrees) {
        // Assuming that a 90-degree turn takes a certain number of encoder ticks.
        // Adjust this value for your robot's specific turning ability.
        int turnTicks = degrees * 20;  // Example multiplier; adjust based on your robot's turn sensitivity

        // Reset encoders before turning
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run to position
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target position (rotate the left motor forward, and right motor backward)
        leftMotor.setTargetPosition(turnTicks);
        rightMotor.setTargetPosition(-turnTicks);  // Reverse the right motor for the turn


        leftMotor.setPower(0.5);  // 50% power
        rightMotor.setPower(0.5);  // 50% power


        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            telemetry.addData("Turning", "Degree: %d", degrees);
            telemetry.update();
            idle();
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}