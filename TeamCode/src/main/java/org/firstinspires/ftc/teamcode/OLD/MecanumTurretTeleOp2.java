package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="MecanumTurretTeleOp", group="Main")
public class MecanumTurretTeleOp2 extends LinearOpMode {

    // --- Hardware Declarations ---
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor turret;
    DcMotor shooter;
    Servo roll1, roll2, roll3;
    DistanceSensor distanceSensor;

    // --- Vision/Parallax Variables ---
    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    // IMPORTANT: These are conceptual offsets. Measure your physical robot accurately in CM/Inches.
    private static final double CAMERA_SHOOTER_X_OFFSET = 5.0; // Horizontal offset (e.g., in cm)
    private static final double CAMERA_SHOOTER_Y_OFFSET = 10.0; // Vertical offset (e.g., in cm)
    // You need your camera's horizontal Field of View (HFOV) to convert pixels to angles.
    private static final double CAMERA_HFOV_DEG = 60.0; // Conceptual HFOV in degrees (check your camera specs)

    // --- State Variables ---
    boolean shooterOn = false;
    boolean shooterButtonPressed = false;
    boolean intakeOn = false;
    boolean intakeButtonPressed = false;
    boolean autoAimMode = false;
    boolean autoAimButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Initialization ---
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");
        turret = hardwareMap.dcMotor.get("turret");
        shooter = hardwareMap.dcMotor.get("shooter");
        roll1 = hardwareMap.get(Servo.class, "roll1");
        roll2 = hardwareMap.get(Servo.class, "roll2");
        roll3 = hardwareMap.get(Servo.class, "roll3");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // --- Hardware Directions and Behavior ---
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Vision Initialization ---
        targetProcessor = new MyTargetProcessor(); // Initialize your new custom processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            // --- Mecanum Drive Control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);

            // --- Auto Aim Toggle ---
            if (gamepad2.y && !autoAimButtonPressed) {
                autoAimMode = !autoAimMode;
                autoAimButtonPressed = true;
            }
            if (!gamepad2.y) autoAimButtonPressed = false;

            // --- Turret Control (Manual or Auto) ---
            if (autoAimMode) {
                aimWithParallaxCorrection(); // Call the auto aim function
            } else {
                // Manual Turret Control
                if (gamepad2.a) {
                    turret.setPower(0.4);
                } else if (gamepad2.left_bumper) {
                    turret.setPower(-0.4);
                } else {
                    turret.setPower(0);
                }
            }

            // --- Shooter and Intake Controls (from original code) ---
            if (gamepad2.b && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterButtonPressed = true;
            }
            if (!gamepad2.b) shooterButtonPressed = false;
            shooter.setPower(shooterOn ? 1.0 : 0.0);

            if (gamepad2.x && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.x) intakeButtonPressed = false;
            double rollPower = intakeOn ? 1.0 : 0.0;
            roll1.setPosition(rollPower);
            roll2.setPosition(rollPower);
            roll3.setPosition(rollPower);

            // --- Telemetry Updates ---
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Auto Aim", autoAimMode ? "ON" : "OFF");
            telemetry.update();
        }
    }

    /**
     * Calculates the aiming correction needed to overcome parallax error
     * and sends commands to the turret motor.
     */
    private void aimWithParallaxCorrection() {
        if (targetProcessor.targetDetected()) {
            double targetPixelX = targetProcessor.getTargetX();
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

            if (Double.isNaN(distanceCM) || distanceCM > 200.0) { // Add a max range check
                telemetry.addData("Aim Status", "Distance out of range or invalid");
                turret.setPower(0);
                return;
            }

            int cameraWidth = targetProcessor.cameraWidth;

            // 1. Convert pixel location to an angle relative to the center of the camera view
            double anglePerPixel = CAMERA_HFOV_DEG / cameraWidth;
            double centerPixelX = cameraWidth / 2.0;
            double angleOffsetFromCenter = (targetPixelX - centerPixelX) * anglePerPixel;

            // 2. Calculate the physical offset angle required due to parallax
            // Use trigonometry: tan(theta) = opposite (offset) / adjacent (distance)
            double parallaxAngleCorrectionDeg = Math.toDegrees(Math.atan(CAMERA_SHOOTER_X_OFFSET / distanceCM));

            // Determine if the target is left or right of center to apply correction correctly.
            // You will need to test this direction on your physical robot.
            double finalTargetAngleDeg = angleOffsetFromCenter + parallaxAngleCorrectionDeg;

            // 3. Command Turret Movement
            // Basic power control (needs a proper PID controller for accuracy):
            if (Math.abs(finalTargetAngleDeg) > 1.0) { // If angle is significant enough to move
                turret.setPower(Math.signum(finalTargetAngleDeg) * 0.3); // Adjust power as needed
            } else {
                turret.setPower(0);
            }

            telemetry.addData("Aim Status", "Correcting Aim");
            telemetry.addData("Target Angle Deg", finalTargetAngleDeg);
            telemetry.addData("Distance CM", distanceCM);

        } else {
            telemetry.addData("Aim Status", "Target not found, stopping turret");
            turret.setPower(0);
        }
    }
}

