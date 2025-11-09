package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Continuous Sorter", group = "Test")
public class SorterMech3 extends LinearOpMode {

    private Servo sorter;
    private ColorSensor CS;

    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private String targetColor = "GREEN";

    @Override
    public void runOpMode() {

        sorter = hardwareMap.get(Servo.class, "sorterServo");
        CS = hardwareMap.get(ColorSensor.class, "colorSensor");

        ATP = AprilTagProcessor.easyCreateWithDefaults();
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                ATP
        );

        telemetry.addLine("Waiting");
        telemetry.update();
        waitForStart();

        AprilTagDetection tag = null;
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && tag == null && System.currentTimeMillis() - startTime < 3000) {
            List<AprilTagDetection> detections = ATP.getDetections();

            if (!detections.isEmpty()) {
                tag = detections.get(0);
            }

            telemetry.addLine("Scanning");
            telemetry.update();
            sleep(100);
        }

        if (tag != null) {
            int tagId = tag.id;

            if (tagId == 1) {
                targetColor = "PURPLE";
            } else if (tagId == 2) {
                targetColor = "GREEN";
            }

            telemetry.addData("Detected Tag ID", tag.id);
        } else {
            telemetry.addLine("No tag detected. Using default color.");
        }

        telemetry.addData("Target Color", targetColor);
        telemetry.update();

        sleep(1000);

        telemetry.addLine("Now sorting continuously...");
        telemetry.update();

        while (opModeIsActive()) {

            String detectedColor = detectColor(CS);

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Target Color", targetColor);

            if (detectedColor.equals("PURPLE")) {
                sorter.setPosition(1.0);
                telemetry.addLine(" PURPLE detected: moving right");
            } else if (detectedColor.equals("GREEN")) {
                sorter.setPosition(0);
                telemetry.addLine(" GREEN detected: moving left");
            }

            telemetry.update();

            sleep(1000);
        }

        telemetry.addLine("Stopped");
        telemetry.update();
    }

    private String detectColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        if (red > green || blue > green) {
            return "PURPLE";
        } else if (green > red && green > blue && green>350) {
            return "GREEN";
        }

        return "UNKNOWN";
    }
}

