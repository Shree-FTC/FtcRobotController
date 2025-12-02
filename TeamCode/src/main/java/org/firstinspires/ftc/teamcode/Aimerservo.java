package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.MyTargetProcessor;

@TeleOp(name="Aimerservo", group="Utility")
public class Aimerservo extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    Servo turret;

    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    private static final double CAMERA_HFOV_DEG = 120.0;
    double turretPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------------
        // Drivetrain Setup
        // ------------------------
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------
        // Turret Setup
        // ------------------------
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(turretPos);

        // ------------------------
        // Vision Setup
        // ------------------------
        targetProcessor = new MyTargetProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        telemetry.addLine("READY - waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ------------------------
            // Drivetrain Control
            // ------------------------
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);

            telemetry.addLine("=== Drive ===");
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("FL", frontLeft.getPower());
            telemetry.addData("FR", frontRight.getPower());
            telemetry.addData("BL", backLeft.getPower());
            telemetry.addData("BR", backRight.getPower());

            // ------------------------
            // Vision / Turret Alignment
            // ------------------------
            telemetry.addLine("\n=== Vision Aim ===");

            if (targetProcessor.targetDetected()) {

                double px = targetProcessor.getTargetX();
                int width = targetProcessor.cameraWidth;

                double anglePerPixel = CAMERA_HFOV_DEG / width;
                double center = width / 2.0;
                double angleError = px - center;

                telemetry.addData("Detected", true);
                telemetry.addData("Pixel X", px);
                telemetry.addData("Cam Width", width);
                telemetry.addData("Angle/Pixel", anglePerPixel);
                telemetry.addData("Angle Error (pixels)", angleError);
                telemetry.addData("Servo Pos", turretPos);

                if (angleError > 0)
                    telemetry.addData("Turn", "RIGHT to align");
                else
                    telemetry.addData("Turn", "LEFT to align");

                // Auto aim (both controllers)
                if(gamepad2.x) {
                    double k = 0.008;
                    turretPos += angleError * k;
                    turretPos = Math.max(0.0, Math.min(1.0, turretPos));
                }

            } else {
                telemetry.addData("Detected", false);
                telemetry.addData("Info", "No tag in view");
            }

            // Manual fine tuning (gamepad2)
            if (gamepad2.dpad_left)  turretPos -= 0.005;
            if (gamepad2.dpad_right) turretPos += 0.005;

            turretPos = Math.max(0.0, Math.min(1.0, turretPos));
            turret.setPosition(turretPos);

            telemetry.update();
        }
    }
}
