package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="TeleOp 2", group="TeleOp")
public class Tele2 extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private CRServo sorter1, sorter2, sorter3, sorter4;

    private ColorSensor CS;
    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final int GREEN_THRESHOLD = 150;

    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    private String shootingOrder = "UNKNOWN";
    private String ballSequence = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        initializeHardware();

        telemetry.addLine("Scanning for AprilTag...");
        telemetry.update();
        scanAprilTag();

        telemetry.addLine("Ready to start");
        telemetry.addData("Shooting Order", shootingOrder);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveControl();
            updateLocalizationFromTag();
            handleSorting();

            telemetry.addData("Detected Color", detectColor());
            telemetry.addData("Shooting Order", shootingOrder);
            telemetry.addData("Ball Sequence", ballSequence);
            telemetry.addData("X (in)", "%.2f", robotX);
            telemetry.addData("Y (in)", "%.2f", robotY);
            telemetry.addData("Heading (deg)", "%.2f", robotHeading);
            telemetry.addData("Color", "R:%d G:%d B:%d", CS.red(), CS.green(), CS.blue());
            telemetry.update();
        }
    }

    private void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        sorter1 = hardwareMap.get(CRServo.class, "sorterServo1");
        sorter2 = hardwareMap.get(CRServo.class, "sorterServo2");
        sorter3 = hardwareMap.get(CRServo.class, "sorterServo3");
        sorter4 = hardwareMap.get(CRServo.class, "sorterServo4");
        CS = hardwareMap.get(ColorSensor.class, "colorSensor");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        ATP = AprilTagProcessor.easyCreateWithDefaults();
        VP = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), ATP);

        stopSorters();
    }

    private void driveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lfPower = (y + x + rx) / denominator;
        double lrPower = (y - x + rx) / denominator;
        double rfPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        leftFront.setPower(lfPower * DRIVE_SPEED);
        leftBack.setPower(lrPower * DRIVE_SPEED);
        rightFront.setPower(rfPower * DRIVE_SPEED);
        rightBack.setPower(rrPower * DRIVE_SPEED);
    }

    private void handleSorting() {
        String color = detectColor();

        if (color.equals("GREEN")) {
            setGreenPipe();
        } else {
            setPurplePipe();
        }

        if (gamepad1.a) stopSorters();
    }

    private String detectColor() {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();

        if (green > GREEN_THRESHOLD && green > red && green > blue) {
            return "GREEN";
        }
        return "PURPLE";
    }

    private void setPurplePipe() {
        sorter1.setPower(1);
        sorter2.setPower(-1);
        sorter3.setPower(1);
        sorter4.setPower(-1);
    }

    private void setGreenPipe() {
        sorter1.setPower(-1);
        sorter2.setPower(1);
        sorter3.setPower(-1);
        sorter4.setPower(1);
    }

    private void stopSorters() {
        sorter1.setPower(0);
        sorter2.setPower(0);
        sorter3.setPower(0);
        sorter4.setPower(0);
    }

    private void updateLocalizationFromTag() {
        List<AprilTagDetection> detections = ATP.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            robotX = tag.ftcPose.x; // inches
            robotY = tag.ftcPose.y; // inches
            robotHeading = tag.ftcPose.bearing; // degrees
        }
    }

    private void scanAprilTag() {
        long startTime = System.currentTimeMillis();
        boolean tagFound = false;

        while (System.currentTimeMillis() - startTime < 5000 && !isStarted()) {
            List<AprilTagDetection> detections = ATP.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection detection = detections.get(0);
                int tagId = detection.id;
                tagFound = true;

                switch (tagId) {
                    case 1:
                        shootingOrder = "PURPLE_FIRST";
                        ballSequence = "PURPLE, GREEN, PURPLE";
                        break;
                    case 2:
                        shootingOrder = "GREEN_FIRST";
                        ballSequence = "GREEN, PURPLE, GREEN";
                        break;
                    case 3:
                        shootingOrder = "ALTERNATE";
                        ballSequence = "PURPLE, GREEN, PURPLE, GREEN";
                        break;
                    default:
                        shootingOrder = "TAG_" + tagId;
                        ballSequence = "UNKNOWN SEQUENCE";
                        break;
                }

                telemetry.addData("✓ AprilTag Found!", "ID: %d", tagId);
                telemetry.addData("✓ Shooting Order", shootingOrder);
                telemetry.addData("✓ Ball Sequence", ballSequence);
                telemetry.update();
                sleep(1000);
                break;
            }

            telemetry.addLine("Searching for AprilTag...");
            telemetry.update();
            sleep(100);
        }

        if (!tagFound) {
            shootingOrder = "DEFAULT_ORDER";
            ballSequence = "PURPLE, GREEN, PURPLE";
            telemetry.addLine("✗ NO APRILTAG DETECTED");
            telemetry.addLine("Using default order");
            telemetry.update();
            sleep(500);
        }
    }
}
