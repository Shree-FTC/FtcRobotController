package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="SimpleShooterTeleOpWithTurret3", group="Main")
public class ShooterColorSequenceTeleOp3 extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;

    // Shooter / kicker / sort wheel
    private DcMotor shooter;
    private Servo sortWheel, kicker;

    // Turret servos
    private CRServo turretYaw, turretPitch;

    // Shooter toggle
    private boolean shooterOn = false;
    private double shooterPower = 0.0;
    private boolean shooterButtonPressed = false;

    // Intake toggle
    private boolean intakeOn = false;
    private boolean intakeButtonPressed = false;

    // Kicker press tracking
    private boolean dpadUpPressed = false;

    // Spindexer step tracking - 5 UNIQUE positions (removed duplicates)
    private double[] spindexSteps = {0.3000, 0.7328, 0.0000, 0.05, 0.4828, 0.9161};
    private int spindexIndex = 0;
    private boolean xPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() {

        // ===== Hardware mapping =====
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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);

        kicker.setPosition(1.0);
        sortWheel.setPosition(spindexSteps[spindexIndex]);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===========================
            // INTAKE TOGGLE (A)
            // ===========================
            if (gamepad2.a && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.a) intakeButtonPressed = false;
            rollerMotor.setPower(intakeOn ? 1.0 : 0.0);

            // ===========================
            // DRIVE SYSTEM
            // ===========================
            double y = gamepad2.left_stick_y;
            double x = -gamepad2.left_stick_x;
            double rx = -gamepad2.right_stick_x;

            frontLeft.setPower(clamp(y + x + rx, -1, 1));
            frontRight.setPower(clamp(y - x - rx, -1, 1));
            backLeft.setPower(clamp(y - x + rx, -1, 1));
            backRight.setPower(clamp(y + x - rx, -1, 1));

            // ===========================
            // KICKER (D-PAD UP)
            // ===========================
            if(gamepad2.dpad_up) {
                kicker.setPosition(1.0);
            }
            else {
                kicker.setPosition(0.7);
            }

            // ===========================
            // SHOOTER TOGGLE (B & Y)
            // ===========================
            if ((gamepad2.b || gamepad2.y) && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterPower = gamepad2.b ? 0.85 : 0.75; // B=90%, Y=50%
                shooterButtonPressed = true;
            }
            if (!gamepad2.b && !gamepad2.y) shooterButtonPressed = false;
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            // ===========================
            // SPINDEXER - X cycles through positions
            // ===========================
            if(gamepad2.x && !xPressed){
                spindexIndex++;
                if(spindexIndex >= spindexSteps.length){
                    spindexIndex = 0;
                }
                sortWheel.setPosition(spindexSteps[spindexIndex]);
                xPressed = true;
            }
            if(!gamepad2.x){
                xPressed = false;
            }

            // DPAD_DOWN = +60° independent increments
            if(gamepad2.dpad_down && !dpadDownPressed){
                double currentPos = sortWheel.getPosition();
                double nextPos = currentPos + 0.166;
                if(nextPos > 1.0){
                    nextPos -= 1.0;
                }
                sortWheel.setPosition(nextPos);
                dpadDownPressed = true;
            }
            if(!gamepad2.dpad_down){
                dpadDownPressed = false;
            }

            // ===========================
            // TURRET CONTROL
            // ===========================
            if(gamepad2.right_bumper){
                turretPitch.setPower(0.6);
            } else if(gamepad2.left_bumper){
                turretPitch.setPower(-0.6);
            } else {
                turretPitch.setPower(0);
            }

            if(gamepad2.right_trigger > 0.1){
                turretYaw.setPower(0.6);
            } else if(gamepad2.left_trigger > 0.1){
                turretYaw.setPower(-0.6);
            } else {
                turretYaw.setPower(0);
            }

            // ===========================
            // TELEMETRY - Compact view showing ALL positions
            // ===========================
            telemetry.addData("Shooter", "%s @ %.0f%%", shooterOn ? "ON" : "OFF", shooterPower * 100);
            telemetry.addData("Current Index", "%d/%d", spindexIndex + 1, spindexSteps.length);
            telemetry.addData("Current Pos", "%.4f", sortWheel.getPosition());
            telemetry.addLine("--- All Spindex Positions ---");

            // Show all 8 positions in compact format (2 per line)
            for(int i = 0; i < spindexSteps.length; i += 2) {
                String marker1 = (i == spindexIndex) ? ">" : " ";
                String pos1 = String.format("%s%d:%.4f", marker1, i + 1, spindexSteps[i]);

                if(i + 1 < spindexSteps.length) {
                    String marker2 = (i + 1 == spindexIndex) ? ">" : " ";
                    String pos2 = String.format("%s%d:%.4f", marker2, i + 2, spindexSteps[i + 1]);
                    telemetry.addLine(pos1 + " | " + pos2);
                } else {
                    telemetry.addLine(pos1);
                }
            }

            telemetry.update();
        }
    }

    private static double clamp(double v, double min, double max){
        return Math.max(min, Math.min(max, v));
    }
}