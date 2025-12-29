package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="AutoShootBlueFS", group="Main")
public class AutoShoot5 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;
    DcMotor shooter;

    CRServo turretYaw, turretPitch;
    Servo sortWheel;
    Servo kicker;

    VisionPortal portal;
    MyTargetProcessor processor;

    private static final double CAMERA_HFOV_DEG = 120;
    private static final double PARALLAX_CORRECTION_DEG = 4.5;

    static final double TICKS_PER_REV = 560;
    static final double WHEEL_DIAMETER_IN = 3.78;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    static final double TICKS_PER_DEGREE = 10.5;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shooter");

        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(CRServo.class, "turretPitch");
        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.REVERSE);

        processor = new MyTargetProcessor();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(processor)
                .build();

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();

        driveInches(84, 0.5);
        sleep(300);

        turnDegrees(45,0.4);

        if (processor.targetDetected()) {
            autoAimTurret();
        } else {
            telemetry.addLine("NO TAG FOUND");
            telemetry.update();
        }

        shooter.setPower(1.0);
        sleep(1200);
        fireKicker(kicker);
        OneTwoZerodeg();
        fireKicker(kicker);
        OneTwoZerodeg();
        fireKicker(kicker);
        OneTwoZerodeg();
        shooter.setPower(0);

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
        sleep(2000);
    }

    private void driveInches(double inches, double power) {
        int move = (int)(inches * TICKS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(move);
        frontRight.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        backRight.setTargetPosition(move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) { }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void turnDegrees(double degrees, double power) {
        int move = (int)(degrees * TICKS_PER_DEGREE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        frontRight.setTargetPosition(-move);
        backRight.setTargetPosition(-move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) { }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void autoAimTurret() {
        // Get target coordinates from vision processor
        double targetX = processor.getTargetX();
        double targetY = processor.getTargetY();
        int width = processor.cameraWidth;
        int height = processor.cameraHeight;

        double centerX = width / 2.0;
        double centerY = height / 2.0;

        // Compute offsets in degrees
        double angleX = (targetX - centerX) * CAMERA_HFOV_DEG / width;
        double angleY = (centerY - targetY) * (CAMERA_HFOV_DEG * 9.0 / 16.0) / height; // approx VFOV

        double yawPower = angleX * 0.01;
        double pitchPower = angleY * 0.01;

        // Clamp CRServo power
        yawPower = Math.max(-0.5, Math.min(0.5, yawPower));
        pitchPower = Math.max(-0.5, Math.min(0.5, pitchPower));

        turretYaw.setPower(yawPower);
        turretPitch.setPower(pitchPower);

        sleep(200); // small adjustment window

        turretYaw.setPower(0);
        turretPitch.setPower(0);
    }

    private void fireKicker(Servo kicker) {
        kicker.setPosition(1.0);
        sleep(250);
        kicker.setPosition(0.0);
        sleep(350);
    }

    private void OneTwoZerodeg(){
        sortWheel.setPosition(120.0 / 180.0);
        sleep(240);
        sortWheel.setPosition(0.5);
    }

    private void sixzero(){
        sortWheel.setPosition(60.0 / 180.0);
        sleep(120);
        sortWheel.setPosition(0.5);
    }
}



