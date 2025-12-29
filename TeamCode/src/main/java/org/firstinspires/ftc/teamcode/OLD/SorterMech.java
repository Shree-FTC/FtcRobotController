package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Sorting Test", group = "Test")
public class SorterMech extends LinearOpMode {

    private Servo sorter;
    private ColorSensor CS;

    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private String targetColor = "PURPLE";  // default fallback

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

        while (opModeIsActive() && tag == null && System.currentTimeMillis() - startTime < 2000) {
            List<AprilTagDetection> detections = ATP.getDetections();

            if (!detections.isEmpty()) {
                tag = detections.get(0);
            }
            sleep(100);
            telemetry.addLine("Scanning");
            telemetry.update();

        }

        if (tag != null) {
            int tagId = tag.id;
            if (tagId == 1) {
                targetColor = "PURPLE";
            } else if (tagId == 2) {
                targetColor = "GREEN";
            }
            telemetry.addData("Detected Tag ID", tagId);

        }

        telemetry.addData("Target Color", targetColor);
        telemetry.update();


        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            sleep(1000);

            String detectedColor = detectColor(CS);
            telemetry.addData("Detected Color", detectedColor);

            if (detectedColor.equals(targetColor)) {

                sorter.setPosition(0.2);
                telemetry.addLine("Sent to chute a");
            } else {
                sorter.setPosition(0.8);
                telemetry.addLine("Sent to chute b");

            }

            sleep(1000);
            telemetry.addLine("Done sorting");
            telemetry.update();
        }
    }

    private String detectColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        if (red > green && blue > green) {
            return "PURPLE";
        }
        else if (green > red && green > blue) {
            return "GREEN";
        }
        return "UNKNOWN";
    }
}

