package org.firstinspires.ftc.teamcode.Test_Folder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp(name="Intake Test", group="Competition")
public class Intaketest extends LinearOpMode {

    // sortWheel control
    private Servo sortWheel;
    private ColorSensor cs;

    // Intake motor
    private DcMotor roller;

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // Storage positions (where balls sit)
    private static final double SLOT_0_POSITION = 0.25;      // 0 degrees - intake position
    private static final double SLOT_1_POSITION = 0.6928;    // 120 degrees
    private static final double SLOT_2_POSITION = 1.0;       // 240 degrees

    // Shooting positions (where balls are fired from)
    private static final double SHOOT_POSITION_1 = 0.03;     // First ball shooting position
    private static final double SHOOT_POSITION_2 = 0.4728;   // Second ball shooting position
    private static final double SHOOT_POSITION_3 = 0.8961;   // Third ball shooting position

    // Ball storage - tracks what color is in each slot
    private String[] ballSlots = new String[3]; // "Green", "Purple", or null
    private int currentSlot = 0; // Which slot is currently at the intake position
    private int ballCount = 0; // Total number of balls stored

    // Target pattern from AprilTag
    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;

    // Constants
    private static final int SHOOT_DELAY = 375; // ms delay between shooting each ball (adjust as needed)
    private static final int AUTO_ROTATE_DELAY = 450; // ms to wait after detecting before rotating (increased to prevent double counting)
    private static final int COLOR_THRESHOLD = 80; // Minimum color value to detect a ball
    private static final int MAX_BALLS = 3; // Maximum number of balls the sortWheel can hold
    private static final int DETECTION_COOLDOWN = 600; // ms cooldown after detection to prevent double-counting
    private static final double INTAKE_POWER = 0.75; // Power for intake motor (adjust as needed)
    private static final int SERVO_MOVE_DELAY = 750; // ms to wait for servo movements (0.75 second)

    // Toggle states
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean isShooting = false;

    // Auto-detection state
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true; // Controls whether we accept new balls
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();
    private boolean inCooldown = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A: Scan AprilTag Pattern");
        telemetry.addData("Controls", "B: Execute Shoot Sequence");
        telemetry.addData("Controls", "X: Reset/Clear All Balls");
        telemetry.addData("Controls", "Right Trigger: Run Intake");
        telemetry.addData("Info", "Color sensor AUTO-DETECTS balls");
        telemetry.addData("Info", "Wheel rotates automatically");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Control intake motor with right trigger
            controlroller();

            // Continuously monitor color sensor and auto-rotate (only if intake is enabled)
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            handleControls();
            updateTelemetry();
        }
    }

    private void initHardware() {
        // sortWheel
        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        sortWheel.setPosition(SLOT_0_POSITION);

        // Color sensor
        cs = hardwareMap.get(ColorSensor.class, "cs");

        // Intake motor
        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE); // Change to REVERSE if needed

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize ball slots as empty
        ballCount = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();
    }

    private void countBalls() {
        ballCount = 0;
        for (int i = 0; i < 3; i++) {
            if (ballSlots[i] != null) {
                ballCount++;
            }
        }

        // Disable intake when all slots are full
        intakeEnabled = (ballCount < MAX_BALLS);
    }

    private void controlroller() {
        // Right trigger controls intake motor
        if (gamepad1.right_trigger > 0.1 && intakeEnabled && !isShooting) {
            // Run intake at full power when trigger is pressed
            roller.setPower(INTAKE_POWER);
        } else {
            // Stop intake when trigger is released or intake is disabled
            roller.setPower(0);
        }
    }

    private void autoDetectAndRotate() {
        // Check if we already have all balls
        if (ballCount >= MAX_BALLS) {
            // All slots full - stop accepting new balls
            return;
        }

        // Check if we're in cooldown period (prevents double-counting)
        if (inCooldown) {
            if (cooldownTimer.milliseconds() > DETECTION_COOLDOWN) {
                inCooldown = false;
            } else {
                return; // Still in cooldown, ignore sensor
            }
        }

        // Read color sensor continuously
        int red = cs.red();
        int green = cs.green();
        int blue = cs.blue();

        // Check if a ball is present (any significant color value)
        boolean ballPresent = (red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD);

        if (ballPresent && !ballDetectedInSlot && !inCooldown) {
            // New ball detected! Determine color and store it
            String detectedColor = detectBallColor(red, green, blue);

            if (detectedColor != null) {
                ballSlots[currentSlot] = detectedColor;
                ballCount++;
                ballDetectedInSlot = true;
                detectionTimer.reset();

                telemetry.addData("DETECTED", detectedColor + " in Slot " + currentSlot);
                telemetry.addData("Ball Count", ballCount + "/" + MAX_BALLS);
                telemetry.update();

                // Check if we're now full
                if (ballCount >= MAX_BALLS) {
                    intakeEnabled = false;
                    telemetry.addData("STATUS", "ALL BALLS LOADED!");
                    telemetry.update();
                    sleep(375);
                }
            }
        }

        // After detection, wait a moment then rotate to next slot (if not full)
        if (ballDetectedInSlot && detectionTimer.milliseconds() > AUTO_ROTATE_DELAY) {
            if (ballCount < MAX_BALLS) {
                rotatesortWheelToNextSlot();
                // Start cooldown after rotation to prevent detecting the same ball again
                inCooldown = true;
                cooldownTimer.reset();
            }
            ballDetectedInSlot = false; // Reset for next detection
        }
    }

    private String detectBallColor(int red, int green, int blue) {
        // Determine ball color based on RGB values
        // Adjust these thresholds based on your color sensor and lighting

        if (green > red && green > blue && green > COLOR_THRESHOLD) {
            return "Green";
        } else if ((red > green && blue > green && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))
                || (red + blue > green * 2 && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))) {
            return "Purple";
        }

        return null; // No clear color detected
    }

    private void handleControls() {
        // Scan AprilTag pattern with A button
        if (gamepad1.a) {
            scanAprilTagPattern();
            sleep(225); // Debounce
        }

        // Shoot sequence with B button (with proper debouncing)
        boolean currentBState = gamepad1.b;
        if (currentBState && !lastBState && !isShooting) {
            if (targetPattern != null && ballCount == MAX_BALLS) {
                executeShootSequence();
            } else if (targetPattern == null) {
                telemetry.addData("Error", "No target pattern! Press A to scan AprilTag");
                telemetry.update();
                sleep(750);
            } else if (ballCount < MAX_BALLS) {
                telemetry.addData("Error", "Not enough balls! Have " + ballCount + ", need " + MAX_BALLS);
                telemetry.update();
                sleep(750);
            }
        }
        lastBState = currentBState;

        // Reset/Clear all balls with X button
        boolean currentXState = gamepad1.x;
        if (currentXState && !lastXState && !isShooting) {
            resetBalls();
        }
        lastXState = currentXState;
    }

    private boolean canFulfillPattern() {
        if (targetPattern == null) return false;

        // Create a copy of ballSlots to test if we can build shootOrder
        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

        // Try to build the shoot order the same way executeShootSequence does
        for (int patternIndex = 0; patternIndex < 3; patternIndex++) {
            String requiredColor = targetPattern[patternIndex];
            boolean foundColor = false;

            // Find which slot has the required color (that hasn't been used yet)
            for (int slot = 0; slot < 3; slot++) {
                if (availableBalls[slot] != null && availableBalls[slot].equals(requiredColor)) {
                    availableBalls[slot] = null; // Mark as used
                    foundColor = true;
                    break;
                }
            }

            if (!foundColor) {
                // Can't fulfill this pattern position
                return false;
            }
        }

        return true; // We can build the full shootOrder
    }

    private String getCurrentBallsString() {
        int greenCount = 0;
        int purpleCount = 0;

        for (String color : ballSlots) {
            if (color != null) {
                if (color.equals("Green")) greenCount++;
                else if (color.equals("Purple")) purpleCount++;
            }
        }

        return greenCount + " Green, " + purpleCount + " Purple";
    }

    private void resetBalls() {
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
        ballCount = 0;
        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        currentSlot = 0;
        sortWheel.setPosition(SLOT_0_POSITION);

        telemetry.addData("RESET", "All balls cleared! Ready for intake");
        telemetry.update();
        sleep(750);
    }

    private void scanAprilTagPattern() {
        telemetry.addData("Scanning", "Looking for AprilTag pattern...");
        telemetry.update();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No valid data from Limelight");
            telemetry.update();
            sleep(750);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Error", "No AprilTags detected");
            telemetry.update();
            sleep(750);
            return;
        }

        // Find pattern tags (21, 22, 23)
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                target = fiducial;
                detectedAprilTagId = id;
                setTargetPattern(id);
                break;
            }
        }

        if (target == null) {
            telemetry.addData("Error", "No pattern tags (21, 22, 23) found");
            telemetry.update();
            sleep(750);
            return;
        }

        telemetry.addData("Success!", "Pattern loaded from Tag " + detectedAprilTagId);
        telemetry.addData("Pattern", getPatternString());
        telemetry.addData("You need", getPatternRequirements());
        telemetry.update();
        sleep(1500);
    }

    private String getPatternRequirements() {
        if (targetPattern == null) return "Unknown";

        int greenCount = 0;
        int purpleCount = 0;

        for (String color : targetPattern) {
            if (color.equals("Green")) greenCount++;
            else if (color.equals("Purple")) purpleCount++;
        }

        return greenCount + " Green, " + purpleCount + " Purple";
    }

    private void rotatesortWheelToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;

        switch (currentSlot) {
            case 0:
                sortWheel.setPosition(SLOT_0_POSITION);
                break;
            case 1:
                sortWheel.setPosition(SLOT_1_POSITION);
                break;
            case 2:
                sortWheel.setPosition(SLOT_2_POSITION);
                break;
        }

        telemetry.addData("sortWheel", "Auto-rotated to Slot " + currentSlot);
        telemetry.update();
    }

    private void setTargetPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21:
                targetPattern = new String[]{"Green", "Purple", "Purple"};
                break;
            case 22:
                targetPattern = new String[]{"Purple", "Green", "Purple"};
                break;
            case 23:
                targetPattern = new String[]{"Purple", "Purple", "Green"};
                break;
            default:
                targetPattern = null;
        }
    }

    private void executeShootSequence() {
        if (isShooting) {
            return;
        }

        isShooting = true;
        roller.setPower(0);
        intakeEnabled = false;
        inCooldown = true;

        telemetry.addData("Starting", "Shoot sequence...");
        telemetry.addData("Pattern Order", getPatternString());
        telemetry.addData("Info", "Calculating optimal shoot order...");
        telemetry.update();
        sleep(1125);

        // Create a copy of ballSlots to track which ones we've used
        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

        // Determine shoot order: which slot to shoot for each pattern position
        int[] shootOrder = new int[3];
        boolean canMatchPattern = true;

        // Try to build the ideal shoot order based on pattern
        for (int patternIndex = 0; patternIndex < 3; patternIndex++) {
            String requiredColor = targetPattern[patternIndex];
            int slotWithRequiredBall = -1;

            // Find which slot has the required color (that hasn't been used yet)
            for (int slot = 0; slot < 3; slot++) {
                if (availableBalls[slot] != null && availableBalls[slot].equals(requiredColor)) {
                    slotWithRequiredBall = slot;
                    availableBalls[slot] = null; // Mark as used in the COPY only
                    break;
                }
            }

            if (slotWithRequiredBall == -1) {
                canMatchPattern = false;
                break;
            }

            shootOrder[patternIndex] = slotWithRequiredBall;
        }

        // If we couldn't match the pattern, use failsafe order (0→1→2)
        if (!canMatchPattern) {
            telemetry.addData("WARNING", "Insufficient balls for pattern!");
            telemetry.addData("Failsafe", "Shooting in slot order: 0→1→2");
            telemetry.update();
            sleep(1500);

            shootOrder[0] = 0;
            shootOrder[1] = 1;
            shootOrder[2] = 2;
        }

        // Now shoot in the determined order
        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            telemetry.addData("===", "SHOOTING POSITION " + (shootIndex + 1) + " OF 3 ===");
            telemetry.update();
            sleep(375);

            int slotToShoot = shootOrder[shootIndex];
            String ballColor = ballSlots[slotToShoot];

            if (ballColor == null) {
                telemetry.addData("ERROR", "Slot " + slotToShoot + " is empty!");
                telemetry.update();
                sleep(2250);
                continue;
            }

            telemetry.addData("Shooting", ballColor + " from Slot " + slotToShoot);
            telemetry.addData("Position", (shootIndex + 1) + " of 3");
            telemetry.update();
            sleep(750);

            double shootPosition = getShootPositionForIndex(shootIndex);

            telemetry.addData("Moving", "Slot " + slotToShoot + " to shoot position " + (shootIndex + 1));
            telemetry.addData("Servo Position", String.format("%.4f", shootPosition));
            telemetry.update();

            sortWheel.setPosition(shootPosition);
            sleep(SERVO_MOVE_DELAY * 2);

            telemetry.addData("FIRING!", ballColor + " from Slot " + slotToShoot);
            telemetry.update();
            sleep(SHOOT_DELAY);

            ballSlots[slotToShoot] = null; // Clear from actual storage AFTER firing
            ballCount--;

            telemetry.addData("Complete", "Position " + (shootIndex + 1) + " fired!");
            telemetry.update();
            sleep(750);
        }

        // Return to home position
        telemetry.addData("===", "SEQUENCE COMPLETE ===");
        telemetry.addData("Action", "Returning to home position");
        telemetry.update();
        sleep(750);

        sortWheel.setPosition(SLOT_0_POSITION);
        currentSlot = 0;
        sleep(SERVO_MOVE_DELAY);

        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        isShooting = false;

        telemetry.addData("SUCCESS!", "Shot pattern: " + getPatternString());
        telemetry.addData("Status", "Ready to load new balls");
        telemetry.update();
        sleep(1500);
    }


    private double getShootPositionForIndex(int shootIndex) {
        // Return the shooting position based on which shot this is (0, 1, or 2)
        switch (shootIndex) {
            case 0:
                return SHOOT_POSITION_1;
            case 1:
                return SHOOT_POSITION_2;
            case 2:
                return SHOOT_POSITION_3;
            default:
                return SHOOT_POSITION_1;
        }
    }

    private void movesortWheelToSlot(int slot) {
        currentSlot = slot;
        switch (slot) {
            case 0:
                sortWheel.setPosition(SLOT_0_POSITION);
                break;
            case 1:
                sortWheel.setPosition(SLOT_1_POSITION);
                break;
            case 2:
                sortWheel.setPosition(SLOT_2_POSITION);
                break;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s, %s, %s]", targetPattern[0], targetPattern[1], targetPattern[2]);
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Ball Count", ballCount + "/" + MAX_BALLS);
        telemetry.addData("Intake", intakeEnabled ? "ACTIVE" : "FULL - DISABLED");
        telemetry.addData("Intake Motor", roller.getPower() > 0 ? "RUNNING" : "STOPPED");

        if (inCooldown) {
            telemetry.addData("Detection", "COOLDOWN (" + (int)(DETECTION_COOLDOWN - cooldownTimer.milliseconds()) + "ms)");
        } else {
            telemetry.addData("Detection", "READY");
        }

        // Real-time color sensor readings
        telemetry.addData("---", "Live Color Sensor ---");
        telemetry.addData("Red", cs.red());
        telemetry.addData("Green", cs.green());
        telemetry.addData("Blue", cs.blue());
        telemetry.addData("Ball Present", (cs.red() > COLOR_THRESHOLD || cs.green() > COLOR_THRESHOLD || cs.blue() > COLOR_THRESHOLD) ? "YES" : "NO");

        telemetry.addData("---", "Ball Storage ---");
        telemetry.addData("Slot 0", ballSlots[0] != null ? ballSlots[0] : "Empty");
        telemetry.addData("Slot 1", ballSlots[1] != null ? ballSlots[1] : "Empty");
        telemetry.addData("Slot 2", ballSlots[2] != null ? ballSlots[2] : "Empty");

        telemetry.addData("---", "Target Pattern ---");
        if (targetPattern != null) {
            telemetry.addData("Tag ID", detectedAprilTagId);
            telemetry.addData("Pattern", getPatternString());
            telemetry.addData("Need", getPatternRequirements());
            telemetry.addData("Have", getCurrentBallsString());
            telemetry.addData("Can Shoot?", canFulfillPattern() ? "YES" : "NO - Wrong colors!");
        } else {
            telemetry.addData("Pattern", "Not scanned (Press A)");
        }

        telemetry.addData("---", "---");
        telemetry.addData("Shooting", isShooting ? "YES" : "NO");
        telemetry.update();
    }
}
