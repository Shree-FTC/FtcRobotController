package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "encoder2", group = "Linear Opmode")
public class encoder2 extends LinearOpMode {

    private DcMotor armMotor;

    @Override
    public void runOpMode() {
        // Initialize the motor from the hardware map
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Reset the encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motor to use encoders
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the target position and speed
        int targetPosition = 1000; // Adjust based on your mechanism
        double power = 0.5; // Adjust speed as needed

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Check if the B button is pressed
            if (gamepad1.b) {
                armMotor.setTargetPosition(targetPosition); // Set the target position
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Start moving the motor to the target position
                armMotor.setPower(power); // Set the power to move the motor

                // Wait until the motor reaches the target position
                while (opModeIsActive() && armMotor.isBusy()) {
                    telemetry.addData("Target Position", targetPosition);
                    telemetry.addData("Current Position", armMotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop the motor once the target is reached
                armMotor.setPower(0);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Return to using the encoder
            }

            // Display telemetry data during runtime
            telemetry.addData("Motor Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
