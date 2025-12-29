package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@TeleOp(name="SimpleShooterTeleOpWithTurret5", group="Main")
public class ShooterColorSequenceTeleOp5 extends LinearOpMode {

    // ===========================
    // HARDWARE
    // ===========================
    private DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;
    private DcMotor shooter;
    private Servo sortWheel, kicker;
    private CRServo turretYaw, turretPitch;
    private HuskyLens huskyLens;

    // ===========================
    // SHOOTER
    // ===========================
    private boolean shooterOn = false;
    private double shooterPower = 0.0;
    private boolean shooterButtonPressed = false;

    // Intake
    private boolean intakeOn = false;
    private boolean intakeButtonPressed = false;

    // Spindexer
    private double[] spindexSteps = {0.0, 0.333, 0.667};
    private int spindexIndex = 0;
    private boolean xPressed = false;
    private boolean dpadDownPressed = false;

    // ===========================
    // AUTO AIM CONSTANTS
    // ===========================
    private static final double CAMERA_OFFSET_X = 3.0; // inches (camera left of turret)
    private static final double TURRET_KP = 0.02;
    private static final double DEADZONE_DEG = 1.0;

    private static final double HFOV_DEG = 120.0;
    private static final int IMAGE_WIDTH = 320;

    @Override
    public void runOpMode() {

        // ===========================
        // MAP HARDWARE
        // ===========================
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");

        shooter = hardwareMap.dcMotor.get("shooter");
        sortWheel = hardwareMap.servo.get("sortWheel");
        kicker = hardwareMap.servo.get("kicker");

        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(CRServo.class, "turretPitch");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        sortWheel.setPosition(spindexSteps[0]);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        // ===========================
        // MAIN LOOP
        // ===========================
        while (opModeIsActive()) {

            // ---------------------------
            // INTAKE TOGGLE (A)
            // ---------------------------
            if (gamepad2.a && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.a) intakeButtonPressed = false;
            rollerMotor.setPower(intakeOn ? 1.0 : 0.0);

            // ---------------------------
            // DRIVE
            // ---------------------------
            double y = gamepad2.left_stick_y;
            double x = -gamepad2.left_stick_x;
            double rx = -gamepad2.right_stick_x;

            frontLeft.setPower(clamp(y + x + rx, -1, 1));
            frontRight.setPower(clamp(y - x - rx, -1, 1));
            backLeft.setPower(clamp(y - x + rx, -1, 1));
            backRight.setPower(clamp(y + x - rx, -1, 1));

            // ---------------------------
            // SHOOTER TOGGLE (B / Y)
            // ---------------------------
            if ((gamepad2.b || gamepad2.y) && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterPower = gamepad2.b ? 0.9 : 0.5;
                shooterButtonPressed = true;
            }
            if (!gamepad2.b && !gamepad2.y) shooterButtonPressed = false;
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            // ---------------------------
            // SPINDEXER
            // ---------------------------
            if (gamepad2.x && !xPressed) {
                spindexIndex++;
                if (spindexIndex >= spindexSteps.length) spindexIndex = 0;
                sortWheel.setPosition(spindexSteps[spindexIndex]);
                xPressed = true;
            }
            if (!gamepad2.x) xPressed = false;

            if (gamepad2.dpad_down && !dpadDownPressed) {
                sortWheel.setPosition(sortWheel.getPosition() + 0.166); // +60°
                dpadDownPressed = true;
            }
            if (!gamepad2.dpad_down) dpadDownPressed = false;

            // ---------------------------
            // TURRET PITCH (MANUAL)
            // ---------------------------
            if (gamepad2.right_bumper) turretPitch.setPower(0.6);
            else if (gamepad2.left_bumper) turretPitch.setPower(-0.6);
            else turretPitch.setPower(0);

            // ---------------------------
            // TURRET YAW (AUTO AIM)
            // ---------------------------
            double yawPower = 0;

            if (shooterOn) {
                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {
                    HuskyLens.Block tag = blocks[0];

                    double pixelOffset = tag.x - (IMAGE_WIDTH / 2.0);
                    double rawYawDeg =
                            (pixelOffset / (IMAGE_WIDTH / 2.0)) * (HFOV_DEG / 2.0);

                    double distanceIn = 24.0 / (tag.width / 50.0);

                    double correctedYaw =
                            parallaxCorrectYaw(rawYawDeg, distanceIn);

                    if (Math.abs(correctedYaw) > DEADZONE_DEG) {
                        yawPower = clamp(correctedYaw * TURRET_KP, -0.6, 0.6);
                    }
                }
            } else {
                if (gamepad2.right_trigger > 0.1) yawPower = 0.6;
                else if (gamepad2.left_trigger > 0.1) yawPower = -0.6;
            }

            turretYaw.setPower(yawPower);

            telemetry.addData("Shooter", shooterOn);
            telemetry.addData("Yaw Power", yawPower);
            telemetry.update();
        }
    }

    // ===========================
    // PARALLAX CORRECTION
    // ===========================
    private double parallaxCorrectYaw(double rawYawDeg, double distanceIn) {
        double yawRad = Math.toRadians(rawYawDeg);

        double xCam = distanceIn * Math.sin(yawRad);
        double zCam = distanceIn * Math.cos(yawRad);

        double xTurret = xCam - CAMERA_OFFSET_X;

        return Math.toDegrees(Math.atan2(xTurret, zCam));
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
