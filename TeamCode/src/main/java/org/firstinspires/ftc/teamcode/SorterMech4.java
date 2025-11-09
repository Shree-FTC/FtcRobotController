package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Intake Mechanism", group="Linear Opmode")
public class SorterMech4 extends LinearOpMode {

    private Servo sorter;
    private ColorSensor CS;
    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private static final double PURPLE_PIPE_POSITION = 0.0;
    private static final double GREEN_PIPE_POSITION = 1.0;
    private static final double NEUTRAL_POSITION = 0.5;

    private static final int PURPLE_THRESHOLD_RED = 100;
    private static final int PURPLE_THRESHOLD_BLUE = 150;
    private static final int GREEN_THRESHOLD_GREEN = 150;

    private String shootingOrder = "UNKNOWN";
    private String ballSequence = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        initializeHardware();

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Scanning for AprilTag...");
        telemetry.update();

        scanAprilTag();

        telemetry.addLine("Ready to start");
        telemetry.addData("Shooting Order", shootingOrder);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            String ballColor = detectBallColor();

            if (ballColor.equals("GREEN")) {
                sorter.setPosition(GREEN_PIPE_POSITION);
                telemetry.addData("Status", "Sorting GREEN ball");
                sorter.setPosition(0.8);
            } else {
                sorter.setPosition(PURPLE_PIPE_POSITION);
                telemetry.addData("Status", "Sorting PURPLE ball");
                sorter.setPosition(0.2);
            }

            telemetry.addData("Ball Detected", ballColor);
            telemetry.addData("Shooting Order", shootingOrder);
            telemetry.addData("Ball Sequence", ballSequence);
            telemetry.addData("Servo Position", "%.2f", sorter.getPosition());
            telemetry.addData("Color - Red", CS.red());
            telemetry.addData("Color - Green", CS.green());
            telemetry.addData("Color - Blue", CS.blue());
            telemetry.update();

            sleep(100);
        }
    }

    private void initializeHardware() {
        sorter = hardwareMap.get(Servo.class, "sorterServo");
        CS = hardwareMap.get(ColorSensor.class, "colorSensor");

        ATP = AprilTagProcessor.easyCreateWithDefaults();
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                ATP
        );

        sorter.setPosition(NEUTRAL_POSITION);

        telemetry.addLine("Waiting");
        telemetry.update();
    }

    private String detectBallColor() {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();

        if (green > GREEN_THRESHOLD_GREEN && green > red && green > blue) {
            return "GREEN";
        }

        return "PURPLE";
    }

    private void scanAprilTag() {
        sleep(500);

        telemetry.addLine("Starting AprilTag scan...");
        telemetry.addLine("Camera should be active now");
        telemetry.update();

        long startTime = System.currentTimeMillis();
        boolean tagFound = false;

        while (System.currentTimeMillis() - startTime < 5000 && !isStarted()) {
            List<AprilTagDetection> detections = ATP.getDetections();

            telemetry.addData("Scan Time", "%.1f seconds", (System.currentTimeMillis() - startTime) / 1000.0);
            telemetry.addData("Tags Detected", detections.size());

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
                telemetry.addLine("Scan complete!");
                telemetry.update();
                sleep(1000);
                break;
            }

            telemetry.addLine("Searching for AprilTag...");
            telemetry.addLine("Point camera at AprilTag");
            telemetry.update();
            sleep(100);
        }

        if (!tagFound) {
            telemetry.addLine("✗ NO APRILTAG DETECTED");
            telemetry.addLine("Using default order");
            shootingOrder = "DEFAULT_ORDER";
            ballSequence = "PURPLE, GREEN, PURPLE";
            telemetry.update();
            sleep(1000);
        }
    }

    public void setPurplePipe() {
        sorter.setPosition(PURPLE_PIPE_POSITION);

    }

    public void setGreenPipe() {
        sorter.setPosition(GREEN_PIPE_POSITION);
    }

    public void setNeutral() {
        sorter.setPosition(NEUTRAL_POSITION);
    }
}