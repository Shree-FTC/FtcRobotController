package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.MyTargetProcessor;

@Autonomous(name="AutoShootRed", group="Main")
public class AutoShoot extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, rollerMotor;

    DcMotor shooter;

    Servo turretServo;
    private double turretPos = 0.5;

    Servo kicker1, kicker2, kicker3;

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

        turretServo = hardwareMap.get(Servo.class, "turretServo");

        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");
        kicker3 = hardwareMap.get(Servo.class, "kicker3");

        processor = new MyTargetProcessor();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(processor)
                .build();

        turretServo.setPosition(turretPos);

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        //FIRST CYCLE
        driveInches(48, 0.5);
        sleep(300);
        if (processor.targetDetected()) {
            autoAimTurret();
        } else {
            telemetry.addLine("NO TAG FOUND — keeping turret centered.");
            telemetry.update();
        }
        shooter.setPower(1.0);
        sleep(1200);
        fireKicker(kicker1);
        fireKicker(kicker2);
        fireKicker(kicker3);
        shooter.setPower(0);

        //SECOND CYCLE
        turnDegrees(-45,0.4);
        rollerMotor.setPower(1);
        driveInches(24,0.5);
        sleep(300);
        driveInches(-24,0.5);
        turnDegrees(45,0.4);
        shooter.setPower(1);
        sleep(300);
        fireKicker(kicker1);
        fireKicker(kicker2);
        fireKicker(kicker3);
        rollerMotor.setPower(0);
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
                        backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving", inches + " in");
            telemetry.update();
        }

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
                        backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("Turning", degrees + " deg");
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void autoAimTurret() {
        double targetX = processor.getTargetX();
        int width = processor.cameraWidth;

        double anglePerPixel = CAMERA_HFOV_DEG / width;
        double offsetAngle = (targetX - (width / 2.0)) * anglePerPixel;

        double finalAngle = offsetAngle + PARALLAX_CORRECTION_DEG;

        turretPos += finalAngle * 0.0008;
        turretPos = Math.max(0.0, Math.min(1.0, turretPos));

        turretServo.setPosition(turretPos);

        telemetry.addData("Auto Aim Angle", finalAngle);
        telemetry.addData("TurretPos", turretPos);
        telemetry.update();

        sleep(300);
    }

    private void fireKicker(Servo kicker) {
        kicker.setPosition(1.0);
        sleep(250);
        kicker.setPosition(0.0);
        sleep(350);
    }
}


