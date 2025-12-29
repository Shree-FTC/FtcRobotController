package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Intake Test", group="Test")
public class Intaketest extends LinearOpMode {

    // Intake/Shooter system
    private DcMotor rollerMotor;
    private DcMotor shooterMotor;
    private Servo kickerServo;

    // Sort wheel / Spindexer
    private Servo sortWheel;
    private ColorSensor colorSensor;

    // Constants
    private static final double KICKER_OPEN = 0.6;
    private static final double KICKER_CLOSED = 0.3;
    private static final double SORT_WHEEL_INCREMENT = 120.0 / 360.0; // 120 degrees in servo range
    private static final double SORT_WHEEL_ALIGN = 60.0 / 360.0; // 60 degrees for alignment

    // Ball tracking
    private enum BallColor { PURPLE, GREEN, UNKNOWN }
    private BallColor[] ballSlots = new BallColor[3];
    private int ballsScanned = 0;
    private boolean sortingComplete = false;

    // PID Controller for sort wheel
    private PIDController sortWheelPID;
    private double sortWheelTargetPos = 0.0;

    // Toggle states
    private boolean rollerActive = false;
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A: Toggle Roller Motor");
        telemetry.addData("Controls", "B: Scan & Sort Ball");
        telemetry.addData("Controls", "X: Kick Ball");
        telemetry.addData("Controls", "Y: Reset Spindexer");
        telemetry.addData("Controls", "D-Pad Up/Down: Manual Spindexer");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Toggle roller motor with A button
            if (gamepad1.a && !lastAState) {
                rollerActive = !rollerActive;
            }
            lastAState = gamepad1.a;

            // Control roller motor
            if (rollerActive) {
                rollerMotor.setPower(1.0);
            } else {
                rollerMotor.setPower(0);
            }

            // Scan and sort ball with B button
            if (gamepad1.b && !lastBState) {
                scanAndSortBalls();
            }
            lastBState = gamepad1.b;

            // Kick ball with X button
            if (gamepad1.x && !lastXState) {
                kickBall();
            }
            lastXState = gamepad1.x;

            // Reset spindexer with Y button
            if (gamepad1.y && !lastYState) {
                resetSpindexer();
            }
            lastYState = gamepad1.y;

            // Manual spindexer control with D-Pad
            if (gamepad1.dpad_up) {
                sortWheelTargetPos += 0.01;
                sortWheelTargetPos = Math.min(1.0, sortWheelTargetPos);
                sortWheel.setPosition(sortWheelTargetPos);
            } else if (gamepad1.dpad_down) {
                sortWheelTargetPos -= 0.01;
                sortWheelTargetPos = Math.max(0.0, sortWheelTargetPos);
                sortWheel.setPosition(sortWheelTargetPos);
            }

            updateTelemetry();
            sleep(20);
        }
    }

    private void initHardware() {
        // Roller motor
        rollerMotor = hardwareMap.get(DcMotor.class, "rollerMotor");

        // Shooter motor
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // Kicker servo
        kickerServo = hardwareMap.get(Servo.class, "kicker");
        kickerServo.setPosition(KICKER_CLOSED);

        // Sort wheel
        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        sortWheel.setPosition(0.0);

        // Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize PID for sort wheel
        sortWheelPID = new PIDController(1.5, 0.0, 0.1);
    }

    private void scanAndSortBalls() {
        if (ballsScanned >= 3) {
            telemetry.addData("Scan Status", "All 3 balls already scanned!");
            telemetry.addData("Info", "Press Y to reset or X to start kicking");
            telemetry.update();

            // Move to alignment position
            sortWheelTargetPos += SORT_WHEEL_ALIGN;
            moveToTargetPosition();
            sortingComplete = true;
            return;
        }

        // Scan current ball
        BallColor detectedColor = detectBallColor();
        ballSlots[ballsScanned] = detectedColor;

        String colorName = detectedColor == BallColor.GREEN ? "GREEN" :
                detectedColor == BallColor.PURPLE ? "PURPLE" : "UNKNOWN";

        telemetry.addData("Ball " + (ballsScanned + 1) + " Scanned", colorName);
        telemetry.update();

        ballsScanned++;

        if (ballsScanned < 3) {
            // Move to next position (120 degrees)
            sortWheelTargetPos += SORT_WHEEL_INCREMENT;
            moveToTargetPosition();
            sleep(300); // Allow time for ball to settle
        } else {
            // All balls scanned, move to alignment position
            sortWheelTargetPos += SORT_WHEEL_ALIGN;
            moveToTargetPosition();
            sortingComplete = true;
            telemetry.addData("Scan Complete", "Ready to kick! Press X");
            telemetry.update();
        }
    }

    private BallColor detectBallColor() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        telemetry.addData("Color Sensor", "R:%d G:%d B:%d", red, green, blue);

        // Purple detection (high red + blue, low green)
        if (red > 80 && blue > 80 && green < 60 && (red + blue) > green * 2) {
            return BallColor.PURPLE;
        }
        // Green detection (high green, lower red and blue)
        else if (green > 100 && green > red * 1.5 && green > blue * 1.5) {
            return BallColor.GREEN;
        }
        return BallColor.UNKNOWN;
    }

    private void moveToTargetPosition() {
        ElapsedTime moveTimer = new ElapsedTime();

        telemetry.addData("Spindexer", "Moving to position %.2f", sortWheelTargetPos);
        telemetry.update();

        while (moveTimer.seconds() < 2.0 && opModeIsActive()) {
            double currentPos = sortWheel.getPosition();
            double error = sortWheelTargetPos - currentPos;

            if (Math.abs(error) < 0.01) break;

            double correction = sortWheelPID.calculate(error);
            double newPos = currentPos + correction;

            // Clamp to servo range
            newPos = Math.max(0.0, Math.min(1.0, newPos));
            sortWheel.setPosition(newPos);

            sleep(20);
        }

        telemetry.addData("Spindexer", "Position reached!");
        telemetry.update();
    }

    private void kickBall() {
        if (!sortingComplete && ballsScanned < 3) {
            telemetry.addData("Kick Failed", "Scan all 3 balls first!");
            telemetry.update();
            return;
        }

        // Open kicker
        kickerServo.setPosition(KICKER_OPEN);
        telemetry.addData("Kicker", "OPEN - Ball kicked!");
        telemetry.update();
        sleep(300);

        // Close kicker
        kickerServo.setPosition(KICKER_CLOSED);
        telemetry.addData("Kicker", "CLOSED");
        telemetry.update();
        sleep(200);

        // Move to next ball position if more balls remain
        if (ballsScanned > 0) {
            sortWheelTargetPos += SORT_WHEEL_INCREMENT;
            if (sortWheelTargetPos > 1.0) {
                sortWheelTargetPos = sortWheelTargetPos % 1.0;
            }
            moveToTargetPosition();
        }
    }

    private void resetSpindexer() {
        telemetry.addData("Reset", "Resetting spindexer...");
        telemetry.update();

        // Reset all tracking variables
        ballsScanned = 0;
        sortingComplete = false;
        sortWheelTargetPos = 0.0;

        // Reset to home position
        sortWheel.setPosition(0.0);
        sortWheelPID.reset();

        // Clear ball slots
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = BallColor.UNKNOWN;
        }

        telemetry.addData("Reset", "Complete! Ready to scan");
        telemetry.update();
        sleep(500);
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Kicker Position", kickerServo.getPosition() > 0.5 ? "OPEN" : "CLOSED");
        telemetry.addData("Spindexer Position", "%.2f", sortWheel.getPosition());
        telemetry.addData("---", "BALL STATUS");
        telemetry.addData("Balls Scanned", ballsScanned + " / 3");
        telemetry.addData("Sorting Complete", sortingComplete);

        if (ballsScanned > 0) {
            for (int i = 0; i < ballsScanned; i++) {
                String colorStr = ballSlots[i] == BallColor.GREEN ? "GREEN" :
                        ballSlots[i] == BallColor.PURPLE ? "PURPLE" : "UNKNOWN";
                telemetry.addData("  Slot " + (i+1), colorStr);
            }
        }

        telemetry.addData("---", "COLOR SENSOR");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());

        telemetry.update();
    }

    // Simple PID Controller
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
}