package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp(name="Auto-Aim Robot (Limelight)", group="Linear Opmode")
public class AutoAimRobot extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system
    private DcMotor shooter;
    private DcMotor rollerMotor;
    private Servo kickerServo;

    // Sort wheel / Spindexer
    private Servo sortWheel;
    private ColorSensor colorSensor;

    // Turret control
    private CRServo turretYaw;
    private DcMotor turretPitch; // 312 RPM motor with encoder for pitch control

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // Constants
    private static final double TURRET_YAW_SPEED = 0.5; // CRServo speed
    private static final double TURRET_PITCH_SPEED = 0.3; // 312 RPM motor speed (adjust as needed)
    private static final double KICKER_OPEN = 0.6;
    private static final double KICKER_CLOSED = 0.3;
    private static final double SORT_WHEEL_INCREMENT = 120.0 / 360.0; // 120 degrees
    private static final double SORT_WHEEL_ALIGN = 60.0 / 360.0; // 60 degrees for alignment

    // Limelight processing
    private static final double TX_GAIN = 0.02; // Proportional gain for horizontal aiming
    private static final double TY_GAIN = 0.02; // Proportional gain for vertical aiming

    // Ball tracking
    private enum BallColor { PURPLE, GREEN, UNKNOWN }
    private BallColor[] ballSlots = new BallColor[3];
    private int ballsScanned = 0;
    private boolean sortingComplete = false;

    // MOTIF tracking (detected from OBELISK AprilTag)
    private BallColor[] targetMotif = new BallColor[3]; // The pattern to build (GPP, PGP, or PPG)
    private boolean motifDetected = false;

    // Toggle states
    private boolean autoAimEnabled = false;
    private boolean lastBState = false;
    private boolean manualShootEnabled = false;
    private boolean lastYState = false;
    private boolean intakeActive = false;
    private boolean lastAState = false;
    private boolean lastXState = false;

    // PID Controller for sort wheel
    private PIDController sortWheelPID;
    private double sortWheelTargetPos = 0.0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Vision", "Limelight 3A");
        telemetry.addData("Controls", "A: Toggle Intake/Scan Ball");
        telemetry.addData("Controls", "B: Toggle Auto-Aim");
        telemetry.addData("Controls", "X: Shoot Sequence");
        telemetry.addData("Controls", "Y: Toggle Manual Shooter");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drive controls (Gamepad 1 only)
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            mecanum(drive, strafe, turn);

            // Toggle auto-aim with B button
            if (gamepad1.b && !lastBState) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastBState = gamepad1.b;

            // Auto-aim or manual turret control
            if (autoAimEnabled) {
                autoAim();
            } else {
                manualTurretControl();
            }

            // Detect MOTIF from OBELISK if not already detected
            if (!motifDetected && autoAimEnabled) {
                detectMotif();
            }

            // Toggle intake/roller and ball scanning with A button
            if (gamepad1.a && !lastAState) {
                intakeActive = !intakeActive;
                if (intakeActive && ballsScanned < 3) {
                    scanAndSortBalls();
                }
            }
            lastAState = gamepad1.a;

            // Control roller motor based on intake state
            if (intakeActive) {
                rollerMotor.setPower(1.0);
            } else {
                rollerMotor.setPower(0);
            }

            // Toggle manual shooter with Y button
            if (gamepad1.y && !lastYState) {
                manualShootEnabled = !manualShootEnabled;
            }
            lastYState = gamepad1.y;

            // Manual shooter control
            if (manualShootEnabled) {
                shooter.setPower(0.8);
            } else if (!autoAimEnabled) {
                shooter.setPower(0);
            }

            // Shooting sequence with X
            if (gamepad1.x && !lastXState) {
                if (sortingComplete) {
                    shootSequence();
                } else if (ballsScanned < 3) {
                    telemetry.addData("Shoot Failed", "Scan all 3 balls first!");
                    telemetry.update();
                }
            }
            lastXState = gamepad1.x;

            updateTelemetry();
            sleep(20);
        }
    }

    private void initHardware() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Shooter system
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        rollerMotor = hardwareMap.get(DcMotor.class, "rollerMotor");
        kickerServo = hardwareMap.get(Servo.class, "kicker");

        // Sort wheel
        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Turret control
        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch"); // 312 RPM motor with encoder

        // Set turret pitch motor to brake mode for better control
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoder for turret pitch
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize positions
        kickerServo.setPosition(KICKER_CLOSED);
        sortWheel.setPosition(0.0);

        // Initialize PID for sort wheel
        sortWheelPID = new PIDController(1.5, 0.0, 0.1);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Switch to AprilTag pipeline
        limelight.start(); // Start polling for data
    }

    private void mecanum(double drive, double strafe, double turn) {
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;

        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max; frPower /= max;
            blPower /= max; brPower /= max;
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void detectMotif() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();

            // OBELISK AprilTag IDs are 21, 22, 23
            if (id == 21) {
                // GPP motif
                targetMotif[0] = BallColor.GREEN;
                targetMotif[1] = BallColor.PURPLE;
                targetMotif[2] = BallColor.PURPLE;
                motifDetected = true;
                break;
            } else if (id == 22) {
                // PGP motif
                targetMotif[0] = BallColor.PURPLE;
                targetMotif[1] = BallColor.GREEN;
                targetMotif[2] = BallColor.PURPLE;
                motifDetected = true;
                break;
            } else if (id == 23) {
                // PPG motif
                targetMotif[0] = BallColor.PURPLE;
                targetMotif[1] = BallColor.PURPLE;
                targetMotif[2] = BallColor.GREEN;
                motifDetected = true;
                break;
            }
        }
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turretYaw.setPower(0);
            turretPitch.setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            turretYaw.setPower(0);
            turretPitch.setPower(0);
            return;
        }

        // Find the GOAL tag (ID 20 for blue, ID 24 for red)
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 20 || id == 24) {
                target = fiducial;
                break;
            }
        }

        // If no GOAL tag found, use any tag
        if (target == null) {
            target = fiducials.get(0);
        }

        // Get target angles from Limelight
        double tx = target.getTargetXDegrees(); // Horizontal offset
        double ty = target.getTargetYDegrees(); // Vertical offset
        double ta = target.getTargetArea(); // Target area for distance

        // Estimate distance based on target area
        double estimatedDistance = 100.0 / Math.sqrt(ta);

        // Calculate turret adjustments
        double yawPower = Math.max(-TURRET_YAW_SPEED, Math.min(TURRET_YAW_SPEED, tx * TX_GAIN));
        double pitchPower = Math.max(-TURRET_PITCH_SPEED, Math.min(TURRET_PITCH_SPEED, -ty * TY_GAIN));

        turretYaw.setPower(yawPower);
        turretPitch.setPower(pitchPower);

        // Calculate shooter power based on distance
        double shooterPower = calculateShooterPower(estimatedDistance);
        shooter.setPower(shooterPower);

        telemetry.addData("Auto-Aim", "Active");
        telemetry.addData("Target ID", (int) target.getFiducialId());
        telemetry.addData("TX", "%.2f", tx);
        telemetry.addData("TY", "%.2f", ty);
        telemetry.addData("Distance", "%.1f in", estimatedDistance);
        telemetry.addData("Pitch Encoder", turretPitch.getCurrentPosition());
        telemetry.addData("Shooter Power", "%.2f", shooterPower);
    }

    private double calculateShooterPower(double distance) {
        // Empirical formula - adjust based on testing
        if (distance < 24) {
            return 0.5;
        } else if (distance < 48) {
            return 0.6 + (distance - 24) * 0.01;
        } else if (distance < 96) {
            return 0.8 + (distance - 48) * 0.005;
        } else {
            return 1.0;
        }
    }

    private void manualTurretControl() {
        // Bumpers control yaw (left/right)
        double yawPower = 0;
        if (gamepad1.left_bumper) {
            yawPower = -TURRET_YAW_SPEED;
        } else if (gamepad1.right_bumper) {
            yawPower = TURRET_YAW_SPEED;
        }

        // Triggers control pitch (up/down) - using 312 RPM motor with encoder
        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) {
            pitchPower = -gamepad1.left_trigger * TURRET_PITCH_SPEED;
        } else if (gamepad1.right_trigger > 0.1) {
            pitchPower = gamepad1.right_trigger * TURRET_PITCH_SPEED;
        }

        turretYaw.setPower(yawPower);
        turretPitch.setPower(pitchPower);
    }

    private void scanAndSortBalls() {
        if (ballsScanned >= 3) {
            telemetry.addData("Scan Status", "All 3 balls already scanned!");
            telemetry.addData("Info", "Press X to start shooting");
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
            telemetry.addData("Scan Complete", "Ready to shoot! Press X");
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

            newPos = Math.max(0.0, Math.min(1.0, newPos));
            sortWheel.setPosition(newPos);

            sleep(20);
        }

        telemetry.addData("Spindexer", "Position reached!");
        telemetry.update();
    }

    private void shootSequence() {
        if (!sortingComplete) {
            telemetry.addData("Shoot Failed", "Complete scanning first!");
            telemetry.update();
            return;
        }

        // Determine shooting order based on detected MOTIF
        int[] shootOrder;

        if (motifDetected) {
            shootOrder = determineShootOrderByMotif();
        } else {
            // Default order if motif not scanned
            shootOrder = new int[]{0, 1, 2};
        }

        for (int i = 0; i < 3; i++) {
            int slotIndex = shootOrder[i];

            // Rotate to correct ball position
            double targetPos = SORT_WHEEL_ALIGN + (slotIndex * SORT_WHEEL_INCREMENT);
            sortWheelTargetPos = targetPos % 1.0;
            moveToTargetPosition();

            // Spin up shooter
            shooter.setPower(0.8);
            sleep(500);

            // Kick ball
            kickerServo.setPosition(KICKER_OPEN);
            telemetry.addData("Kicker", "OPEN - Ball " + (i+1) + " kicked!");
            telemetry.update();
            sleep(300);

            kickerServo.setPosition(KICKER_CLOSED);
            telemetry.addData("Kicker", "CLOSED");
            telemetry.update();
            sleep(300);
        }

        shooter.setPower(0);

        // Reset for next cycle
        sortingComplete = false;
        ballsScanned = 0;
        sortWheelTargetPos = 0.0;
        sortWheel.setPosition(0.0);
        sortWheelPID.reset();

        for (int i = 0; i < 3; i++) {
            ballSlots[i] = BallColor.UNKNOWN;
        }

        telemetry.addData("Shoot Sequence", "Complete! Ready for next cycle");
        telemetry.update();
    }

    private int[] determineShootOrderByMotif() {
        int[] order = new int[3];
        int orderIndex = 0;

        // Match balls to motif positions
        for (int motifIndex = 0; motifIndex < 3; motifIndex++) {
            for (int ballIndex = 0; ballIndex < 3; ballIndex++) {
                if (ballSlots[ballIndex] == targetMotif[motifIndex]) {
                    order[orderIndex++] = ballIndex;
                    break;
                }
            }
        }

        // Fill in any remaining slots
        for (int i = 0; i < 3; i++) {
            boolean alreadyInOrder = false;
            for (int j = 0; j < orderIndex; j++) {
                if (order[j] == i) {
                    alreadyInOrder = true;
                    break;
                }
            }
            if (!alreadyInOrder && orderIndex < 3) {
                order[orderIndex++] = i;
            }
        }

        return order;
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("---", "VISION & AUTO-AIM");
        telemetry.addData("Vision System", "Limelight 3A");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "Disabled");
        telemetry.addData("Pitch Encoder", turretPitch.getCurrentPosition());

        telemetry.addData("---", "SHOOTER SYSTEM");
        telemetry.addData("Manual Shooter", manualShootEnabled ? "ON (0.8)" : "OFF");
        telemetry.addData("Intake/Roller", intakeActive ? "ACTIVE" : "OFF");
        telemetry.addData("Kicker Position", kickerServo.getPosition() > 0.5 ? "OPEN" : "CLOSED");

        telemetry.addData("---", "BALL STATUS");
        telemetry.addData("Balls Scanned", ballsScanned + " / 3");
        telemetry.addData("Sorting Complete", sortingComplete);
        telemetry.addData("Spindexer Position", "%.2f", sortWheel.getPosition());

        // Display detected MOTIF
        if (motifDetected) {
            String motifString = "";
            for (int i = 0; i < 3; i++) {
                motifString += (targetMotif[i] == BallColor.GREEN ? "G" : "P");
            }
            telemetry.addData("MOTIF Detected", motifString);
        } else {
            telemetry.addData("MOTIF", "Not Detected");
        }

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