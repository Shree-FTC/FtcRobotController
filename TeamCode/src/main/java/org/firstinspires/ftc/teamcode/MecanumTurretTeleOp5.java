package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PLEASEBROPLEASE", group="Main")
public class MecanumTurretTeleOp5 extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Other motors
    private DcMotor shooter, intake;

    // Servos
    private Servo kicker;
    private CRServo turretYaw, turretPitch;

    // State tracking
    private boolean intakeOn = false;
    private boolean shooterSpinning = false;

    // Timers
    private ElapsedTime kickTimer = new ElapsedTime();
    private boolean isKicking = false;

    // Button toggles
    private boolean lastA = false;
    private boolean lastX = false;

    @Override
    public void runOpMode() {

        // === MAP ALL HARDWARE ===
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("rollerMotor");
        kicker = hardwareMap.servo.get("kicker");
        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(CRServo.class, "turretPitch");

        // Set motor directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);

        // PREVENT SERVO MOVEMENT DURING INIT
        ServoController kickerCtrl = kicker.getController();
        ServoController pitchCtrl = turretPitch.getController();
        kickerCtrl.pwmDisable();
        pitchCtrl.pwmDisable();

        telemetry.addLine("SimpleTurretBot Ready!");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // NOW ENABLE SERVOS
        kickerCtrl.pwmEnable();
        pitchCtrl.pwmEnable();
        kicker.setPosition(0.1);
        turretYaw.setPower(0);
        turretPitch.setPower(0);

        while (opModeIsActive()) {

            // ==========================================
            // DRIVE - LEFT STICK = MOVE, RIGHT STICK = TURN
            // ==========================================
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            frontLeft.setPower(forward + strafe + turn);
            frontRight.setPower(forward - strafe - turn);
            backLeft.setPower(forward - strafe + turn);
            backRight.setPower(forward + strafe - turn);

            // ==========================================
            // TURRET YAW - LEFT/RIGHT TRIGGERS
            // ==========================================
            double yawPower = gamepad1.right_trigger - gamepad1.left_trigger;
            turretYaw.setPower(yawPower * 0.6);

            // ==========================================
            // TURRET PITCH - LEFT/RIGHT BUMPERS
            // ==========================================
            double pitchPower = 0;
            if (gamepad1.left_bumper) {
                pitchPower = 0.5;  // Up
            } else if (gamepad1.right_bumper) {
                pitchPower = -0.5; // Down
            }
            turretPitch.setPower(pitchPower);

            // ==========================================
            // INTAKE TOGGLE - A BUTTON
            // ==========================================
            if (gamepad1.a && !lastA) {
                intakeOn = !intakeOn;
            }
            lastA = gamepad1.a;

            intake.setPower(intakeOn ? 1.0 : 0.0);

            // ==========================================
            // SHOOTER TOGGLE - X BUTTON
            // ==========================================
            if (gamepad1.x && !lastX) {
                shooterSpinning = !shooterSpinning;
            }
            lastX = gamepad1.x;

            shooter.setPower(shooterSpinning ? 1.0 : 0.0);

            // ==========================================
            // KICK/SHOOT - B BUTTON (HOLD)
            // ==========================================
            if (gamepad1.b && !isKicking) {
                isKicking = true;
                kickTimer.reset();
            }

            if (isKicking) {
                if (kickTimer.seconds() < 0.2) {
                    kicker.setPosition(0.5); // Up
                } else if (kickTimer.seconds() < 0.4) {
                    kicker.setPosition(0.1); // Down
                } else {
                    isKicking = false;
                }
            }

            // ==========================================
            // EMERGENCY STOP - Y BUTTON
            // ==========================================
            if (gamepad1.y) {
                shooter.setPower(0);
                intake.setPower(0);
                shooterSpinning = false;
                intakeOn = false;
            }

            // ==========================================
            // TELEMETRY
            // ==========================================
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Yaw (Triggers)", "%.2f", yawPower);
            telemetry.addData("Pitch (Bumpers)", "%.2f", pitchPower);

            telemetry.addLine();
            telemetry.addLine("=== SYSTEMS ===");
            telemetry.addData("Intake (A)", intakeOn ? "ON" : "OFF");
            telemetry.addData("Shooter (X)", shooterSpinning ? "ON" : "OFF");
            telemetry.addData("Kicking (B)", isKicking ? "YES" : "NO");

            telemetry.update();
        }
    }
}