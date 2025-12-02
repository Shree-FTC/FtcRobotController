package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.MyTargetProcessor;

@TeleOp(name="MecanumTurretTeleOp4", group="Main")
public class MecanumTurretTeleOp4 extends LinearOpMode {

    // --- Hardware ---
    DcMotor frontLeft, frontRight, backLeft, backRight;
    Servo turretServo;
    DcMotor shooter;
    DcMotor rollerMotor;

    // sorter
    Servo sortWheel;
    Servo kicker1, kicker2, kicker3;
    ColorSensor cs;

    // Vision
    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    // Turret
    private double turretPos = 0.5;
    private static final double CAMERA_HFOV_DEG = 60.0;
    private static final double PARALLAX_CORRECTION_DEG = 4.5;

    // Sortwheel slots
    private final double SLOT1_POS = 0.20;
    private final double SLOT2_POS = 0.50;
    private final double SLOT3_POS = 0.80;

    // Kicker positions
    private final double REST_POS = 0.0;
    private final double FIRE_POS = 0.4;

    // States
    boolean shooterOn = false;
    boolean shooterButtonPressed = false;

    boolean intakeOn = false;
    boolean intakeButtonPressed = false;

    boolean autoAimMode = false;
    boolean autoAimButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // drive
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        shooter = hardwareMap.dcMotor.get("shooter");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");

        turretServo = hardwareMap.get(Servo.class, "turretServo");

        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");
        kicker3 = hardwareMap.get(Servo.class, "kicker3");

        cs = hardwareMap.get(ColorSensor.class, "cs");

        // drive direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        kickersRest();

        // setup vision
        targetProcessor = new MyTargetProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        turretServo.setPosition(turretPos);

        waitForStart();

        while (opModeIsActive()) {

            // Drive
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);

            // --- Auto aim toggle ---
            if (gamepad2.y && !autoAimButtonPressed) {
                autoAimMode = !autoAimMode;
                autoAimButtonPressed = true;
            }
            if (!gamepad2.y) autoAimButtonPressed = false;

            // turret
            if (autoAimMode) {
                autoAimTurret();
            } else {
                if (gamepad2.a) turretPos += 0.01;
                if (gamepad2.left_bumper) turretPos -= 0.01;
                turretPos = Math.max(0.0, Math.min(1.0, turretPos));
                turretServo.setPosition(turretPos);
            }

            // shooter
            if (gamepad2.b && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterButtonPressed = true;
            }
            if (!gamepad2.b) shooterButtonPressed = false;

            shooter.setPower(shooterOn ? 1.0 : 0.0);

            // intake
            if (gamepad2.x && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.x) intakeButtonPressed = false;

            rollerMotor.setPower(intakeOn ? 1.0 : 0.0);

            // --- SORT & SHOOT ALL BALLS ---
            if (gamepad2.dpad_up) {
                sortAndShootAll();
            }

            telemetry.addData("Shooter", shooterOn);
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("AutoAim", autoAimMode);
            telemetry.addData("Turret", turretPos);
            telemetry.update();
        }
    }

    // =========================================================
    // AUTO AIM
    // =========================================================
    private void autoAimTurret() {
        if (targetProcessor.targetDetected()) {
            double targetPixelX = targetProcessor.getTargetX();
            int cameraWidth = targetProcessor.cameraWidth;

            double anglePerPixel = CAMERA_HFOV_DEG / cameraWidth;
            double center = cameraWidth / 2.0;

            double angleOffset = (targetPixelX - center) * anglePerPixel;
            double finalAngle = angleOffset + PARALLAX_CORRECTION_DEG;

            turretPos += finalAngle * 0.0005;
            turretPos = Math.max(0.0, Math.min(1.0, turretPos));

            turretServo.setPosition(turretPos);
        }
    }

    // =========================================================
    // SORTING + FIRING MECHANISM
    // =========================================================

    private String readBallColor() {
        int r = cs.red();
        int g = cs.green();

        if (g > r) return "green";
        return "purple";
    }

    private void moveToSlot(double pos) {
        sortWheel.setPosition(pos);
        sleep(300);
    }

    private void fire(Servo kicker) {
        kicker.setPosition(FIRE_POS);
        sleep(300);
        kicker.setPosition(REST_POS);
        sleep(200);
    }

    private void kickersRest() {
        kicker1.setPosition(REST_POS);
        kicker2.setPosition(REST_POS);
        kicker3.setPosition(REST_POS);
    }

    private void sortAndShootAll() {

        // ---- Step 1: Scan all 3 slots ----
        moveToSlot(SLOT1_POS);
        String c1 = readBallColor();

        moveToSlot(SLOT2_POS);
        String c2 = readBallColor();

        moveToSlot(SLOT3_POS);
        String c3 = readBallColor();

        String[] colors = { c1, c2, c3 };

        // ---- Step 2: Decide order ----
        // Same logic as earlier example:
        // green first then purple.
        int[] order = new int[3];
        int idx = 0;

        // green balls first
        for (int i = 0; i < 3; i++)
            if (colors[i].equals("green"))
                order[idx++] = i + 1;

        // purple next
        for (int i = 0; i < 3; i++)
            if (colors[i].equals("purple"))
                order[idx++] = i + 1;

        // ---- Step 3: Fire in that order ----
        for (int slot : order) {
            switch (slot) {
                case 1:
                    moveToSlot(SLOT1_POS);
                    fire(kicker1);
                    break;
                case 2:
                    moveToSlot(SLOT2_POS);
                    fire(kicker2);
                    break;
                case 3:
                    moveToSlot(SLOT3_POS);
                    fire(kicker3);
                    break;
            }
        }
    }
}
