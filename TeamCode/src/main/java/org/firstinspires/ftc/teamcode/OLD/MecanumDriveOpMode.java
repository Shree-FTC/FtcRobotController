package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDriveOpMode extends LinearOpMode {

    // Declare hardware variables
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        leftFront = hardwareMap.get(DcMotor.class, "2"); // top left motor
        rightFront = hardwareMap.get(DcMotor.class, "3"); // top right motor
        leftRear = hardwareMap.get(DcMotor.class, "1"); // bottom left motor
        rightRear = hardwareMap.get(DcMotor.class, "0"); // bottom right motor

        // Set motor directions (assuming motors are wired as per your setup)
        leftFront.setDirection(DcMotor.Direction.FORWARD);  // top left motor
        rightFront.setDirection(DcMotor.Direction.REVERSE); // top right motor
        leftRear.setDirection(DcMotor.Direction.FORWARD);   // bottom left motor
        rightRear.setDirection(DcMotor.Direction.REVERSE);  // bottom right motor

        // Wait for the game to start
        waitForStart();

        // Main loop for controlling the robot
        while (opModeIsActive()) {
            // Get the input from the gamepad
            double x = gamepad1.left_stick_x;  // Left stick X (left-right)
            double y = -gamepad1.left_stick_y; // Left stick Y (forward-backward)
            double rotation = gamepad1.right_stick_x; // Right stick X (rotation)

            // Compute the motor powers for mecanum drive
            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double rearLeftPower = y - x + rotation;
            double rearRightPower = y + x - rotation;

            // Normalize the motor powers to prevent over-driving the motors
            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                rearLeftPower /= max;
                rearRightPower /= max;
            }

            // Set the motor powers
            leftFront.setPower(frontLeftPower);
            rightFront.setPower(frontRightPower);
            leftRear.setPower(rearLeftPower);
            rightRear.setPower(rearRightPower);

            // Optionally, add telemetry for debugging
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.update();
        }
    }
}
