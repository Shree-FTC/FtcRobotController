package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SimpleShooterTeleOpWithTurret", group="Main")
public class ShooterColorSequenceTeleOp extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;

    // Shooter / kicker / sort wheel
    private DcMotor shooter;
    private Servo sortWheel, kicker;

    // Turret servos
    private CRServo turretYaw, turretPitch;

    // Color sensor
    private ColorSensor cs;

    // Shooter toggle
    private boolean shooterOn = false;
    private double shooterPower = 0.0;
    private boolean shooterButtonPressed = false;

    // Intake toggle
    private boolean intakeOn = false;
    private boolean intakeButtonPressed = false;

    // Kicker button press tracker
    private boolean dpadUpPressed = false;

    // Sort wheel position
    private double sortPos = 0.0;

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

        cs = hardwareMap.get(ColorSensor.class, "cs");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        kicker.setPosition(0.1);
        sortWheel.setPosition(sortPos);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===========================
            //    INTAKE TOGGLE (A)
            // ===========================
            if (gamepad2.a && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.a) intakeButtonPressed = false;

            rollerMotor.setPower(intakeOn ? 1.0 : 0.0);


            // ==============
            // DRIVE SYSTEM
            // ==============
            double y = gamepad2.left_stick_y;
            double x = -gamepad2.left_stick_x;
            double rx = -gamepad2.right_stick_x;

            frontLeft.setPower(clamp(y + x + rx, -1, 1));
            frontRight.setPower(clamp(y - x - rx, -1, 1));
            backLeft.setPower(clamp(y - x + rx, -1, 1));
            backRight.setPower(clamp(y + x - rx, -1, 1));


            // ===========================
            //    KICKER (D-PAD UP)
            // ===========================
            if(gamepad2.dpad_up && !dpadUpPressed){
                kicker.setPosition(1.0); // adjust if needed
                sleep(200);               // allow servo to move
                kicker.setPosition(0.0);
                dpadUpPressed = true;
            }
            if(!gamepad2.dpad_up){
                dpadUpPressed = false;
            }


            // ==========================
            // SHOOTER TOGGLE (B & Y)
            // ==========================
            if ((gamepad2.b || gamepad2.y) && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterPower = gamepad2.b ? 0.9 : 0.5;   // B = 90%, Y = 50%
                shooterButtonPressed = true;
            }
            if (!gamepad2.b && !gamepad2.y) shooterButtonPressed = false;

            shooter.setPower(shooterOn ? shooterPower : 0.0);


            // =======================================
            // SORT WHEEL: CONTINUOUS MOVEMENT
            // =======================================
            if (gamepad2.x) {
                sortPos += 0.01; // X = faster
            }
            if (gamepad2.dpad_down) {
                sortPos += 0.005; // DPAD_DOWN = slower
            }
            sortWheel.setPosition(sortPos);


            // ===========================
            //   TURRET CONTROL
            // ===========================

            // Pitch (RB up, LB down)
            if (gamepad2.right_bumper) {
                turretPitch.setPower(0.6);
            } else if (gamepad2.left_bumper) {
                turretPitch.setPower(-0.6);
            } else {
                turretPitch.setPower(0);
            }

            // Yaw (right trigger = right, left trigger = left)
            if (gamepad2.right_trigger > 0.1) {
                turretYaw.setPower(0.6);
            } else if (gamepad2.left_trigger > 0.1) {
                turretYaw.setPower(-0.6);
            } else {
                turretYaw.setPower(0);
            }


            // ===========================
            // TELEMETRY
            // ===========================
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Sort Position", sortPos);
            telemetry.update();
        }
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
