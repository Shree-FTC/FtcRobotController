package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="MecanumTurretTeleOp", group="Main")
public class MecanumTurretTeleOp3 extends LinearOpMode {

    // --- Drive Motors ---
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Turret SERVO ---
    Servo turretServo;

    // --- Shooter ---
    DcMotor shooter;

    // --- Roller (one motor) ---
    DcMotor roller;

    // --- Sorting Mechanism ---
    Servo sortwheel;               // wheel servo with 3 ball slots
    Servo kicker1, kicker2, kicker3; // slot kickers
    ColorSensor cs;                // purple/green sensor

    // 0, 1, 2 = slots for 3 balls
    String[] ballColors = new String[3];
    int ballIndex = 0;
    boolean sortingComplete = false;
    boolean readyToFire = false;
    int[] fireOrder = new int[3];
    String tagMotif = "A"; // your AprilTag processor can fill this

    private final double SLOT_POS_0 = 0.00;
    private final double SLOT_POS_1 = 0.33;
    private final double SLOT_POS_2 = 0.66;

    private final double KICK_POS = 0.75;
    private final double RETRACT_POS = 0.10;

    // --- Vision ---
    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    // Camera offsets / HFOV
    private static final double CAMERA_SHOOTER_X_OFFSET = 5.0;
    private static final double CAMERA_SHOOTER_Y_OFFSET = 10.0;
    private static final double CAMERA_HFOV_DEG = 60.0;

    // States
    boolean shooterOn = false;
    boolean intakeOn = false;
    boolean shooterButtonPressed = false;
    boolean intakeButtonPressed = false;
    boolean autoAimMode = false;
    boolean autoAimButtonPressed = false;

    double turretPos = 0.5;
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Map ---
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        turretServo = hardwareMap.get(Servo.class, "turretServo");
        shooter = hardwareMap.dcMotor.get("shooter");
        roller = hardwareMap.dcMotor.get("roller");

        // Sorting
        sortwheel = hardwareMap.get(Servo.class, "sortwheel");
        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");
        kicker3 = hardwareMap.get(Servo.class, "kicker3");
        cs = hardwareMap.get(ColorSensor.class, "cs");

        kicker1.setPosition(RETRACT_POS);
        kicker2.setPosition(RETRACT_POS);
        kicker3.setPosition(RETRACT_POS);
        sortwheel.setPosition(SLOT_POS_0);

        // Drive direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Vision
        targetProcessor = new MyTargetProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            // --- Mecanum Drive ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);

            // === Auto Aim Toggle ===
            if (gamepad2.y && !autoAimButtonPressed) {
                autoAimMode = !autoAimMode;
                autoAimButtonPressed = true;
            }
            if (!gamepad2.y) autoAimButtonPressed = false;

            // === Turret ===
            if (autoAimMode) {
                aimWithParallaxCorrection();
            } else {
                if (gamepad2.a) turretPos += 0.01;
                if (gamepad2.left_bumper) turretPos -= 0.01;

                turretPos = clamp(turretPos, TURRET_MIN, TURRET_MAX);
                turretServo.setPosition(turretPos);
            }

            // === Shooter toggle ===
            if (gamepad2.b && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterButtonPressed = true;
            }
            if (!gamepad2.b) shooterButtonPressed = false;
            shooter.setPower(shooterOn ? 1.0 : 0.0);

            // === Roller toggle ===
            if (gamepad2.x && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.x) intakeButtonPressed = false;
            roller.setPower(intakeOn ? 1.0 : 0.0);

            // === SORTING LOGIC (press DPAD RIGHT to load next ball) ===
            if (gamepad2.dpad_right && ballIndex < 3) {
                loadNextBall();
            }

            // === FIRING LOGIC (DPAD LEFT to shoot all balls) ===
            if (gamepad2.dpad_left && sortingComplete && !readyToFire) {
                calculateFireOrder();
                readyToFire = true;
            }

            if (readyToFire) {
                fireAllBalls();
                readyToFire = false;
            }

            telemetry.addData("Sorting", ballIndex + "/3");
            telemetry.addData("Colors", "%s %s %s", ballColors[0], ballColors[1], ballColors[2]);
            telemetry.addData("Auto Aim", autoAimMode);
            telemetry.update();
        }
    }

    // ----------------------------------------------
    // SORTING SYSTEM
    // ----------------------------------------------

    private void loadNextBall() {
        double pos = (ballIndex == 0) ? SLOT_POS_0 :
                (ballIndex == 1) ? SLOT_POS_1 : SLOT_POS_2;

        sortwheel.setPosition(pos);
        sleep(300);

        ballColors[ballIndex] = detectColor();
        ballIndex++;

        if (ballIndex == 3) sortingComplete = true;
    }

    private String detectColor() {
        int r = cs.red();
        int g = cs.green();
        int b = cs.blue();

        // green = high green
        if (g > r * 1.3 && g > b * 1.3) return "GREEN";

        // purple = high red and blue
        if (r > g * 1.3 && b > g * 1.3) return "PURPLE";

        return "UNKNOWN";
    }

    private void calculateFireOrder() {

        if (tagMotif.equals("A")) {
            fireOrder[0] = findSlot("GREEN");
            fireOrder[1] = findSlot("PURPLE");
            fireOrder[2] = findRemainingSlot(fireOrder[0], fireOrder[1]);
        }
        else if (tagMotif.equals("B")) {
            fireOrder[0] = findSlot("PURPLE");
            fireOrder[1] = findSlot("GREEN");
            fireOrder[2] = findRemainingSlot(fireOrder[0], fireOrder[1]);
        }
        else {
            fireOrder[0] = 0;
            fireOrder[1] = 1;
            fireOrder[2] = 2;
        }
    }

    private int findSlot(String color) {
        for (int i = 0; i < 3; i++) {
            if (ballColors[i] != null && ballColors[i].equals(color))
                return i;
        }
        return 0;
    }

    private int findRemainingSlot(int a, int b) {
        for (int i = 0; i < 3; i++) {
            if (i != a && i != b) return i;
        }
        return 0;
    }

    private void fireAllBalls() {
        for (int slot : fireOrder) {
            double pos = (slot == 0) ? SLOT_POS_0 :
                    (slot == 1) ? SLOT_POS_1 : SLOT_POS_2;

            sortwheel.setPosition(pos);
            sleep(300);

            kick(slot);
            sleep(400);
        }
    }

    private void kick(int slot) {
        Servo s = (slot == 0) ? kicker1 :
                (slot == 1) ? kicker2 : kicker3;

        s.setPosition(KICK_POS);
        sleep(200);
        s.setPosition(RETRACT_POS);
    }

    // ----------------------------------------------
    // AUTO AIM SERVO LOGIC
    // ----------------------------------------------

    private void aimWithParallaxCorrection() {
        if (!targetProcessor.targetDetected()) {
            turretServo.setPosition(turretPos);
            return;
        }

        double targetX = targetProcessor.getTargetX();
        int width = targetProcessor.cameraWidth;
        double center = width / 2.0;

        double anglePerPixel = CAMERA_HFOV_DEG / width;
        double pixelAngle = (targetX - center) * anglePerPixel;

        double distance = 50; // placeholder if no range sensor used
        double parallaxAngle = Math.toDegrees(Math.atan(CAMERA_SHOOTER_X_OFFSET / distance));

        double finalAngle = pixelAngle + parallaxAngle;

        turretPos += finalAngle / 300.0;
        turretPos = clamp(turretPos, TURRET_MIN, TURRET_MAX);

        turretServo.setPosition(turretPos);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}

