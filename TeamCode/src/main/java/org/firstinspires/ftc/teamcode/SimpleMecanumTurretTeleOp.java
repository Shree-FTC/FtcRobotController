package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ManualTurretTeleOp", group="Main")
public class SimpleMecanumTurretTeleOp extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooter, intake;

    // Servos
    private Servo kicker;
    private CRServo turretYaw, turretPitch;

    // Shooter state
    private boolean shooting = false;
    private ElapsedTime shootTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Map hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("rollerMotor");
        kicker = hardwareMap.servo.get("kicker");
        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(CRServo.class, "turretPitch");

        // Set directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        // DISABLE servos during INIT to prevent movement
        ServoController kickerCtrl = kicker.getController();
        ServoController pitchCtrl = turretPitch.getController();
        kickerCtrl.pwmDisable();
        pitchCtrl.pwmDisable();

        telemetry.addLine("Ready - Press START");
        telemetry.update();

        waitForStart();

        // ENABLE servos and set starting positions
        kickerCtrl.pwmEnable();
        pitchCtrl.pwmEnable();
        kicker.setPosition(0.05);
        turretYaw.setPower(0);
        turretPitch.setPower(0);

        while (opModeIsActive()) {

            // ========== DRIVE ==========
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + turn);
            frontRight.setPower(y - x - turn);
            backLeft.setPower(y - x + turn);
            backRight.setPower(y + x - turn);

            // ========== TURRET CONTROL ==========
            // Pitch control (up/down)
            double pitch = 0;
            if (gamepad1.left_bumper) pitch = 0.5;
            else if (gamepad1.right_bumper) pitch = -0.5;
            turretPitch.setPower(pitch);

            // Yaw control (left/right)
            double yaw = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.5;
            turretYaw.setPower(yaw);

            // ========== INTAKE ==========
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }

            // ========== SHOOTER ==========
            if (gamepad1.b && !shooting) {
                shooting = true;
                shootTimer.reset();
            }

            if (shooting) {
                shooter.setPower(1.0);

                if (shootTimer.seconds() < 0.3) {
                    kicker.setPosition(0.05); // Down
                } else if (shootTimer.seconds() < 0.6) {
                    kicker.setPosition(0.4); // Up (kick)
                } else {
                    kicker.setPosition(0.05); // Down
                    shooter.setPower(0);
                    shooting = false;
                }
            }

            // ========== TELEMETRY ==========
            telemetry.addData("Shooting", shooting ? "YES" : "NO");
            telemetry.addData("Intake", gamepad1.a ? "ON" : "OFF");
            telemetry.addData("Turret Yaw", "%.2f", turretYaw.getPower());
            telemetry.addData("Turret Pitch", "%.2f", turretPitch.getPower());
            telemetry.update();
        }
    }
}