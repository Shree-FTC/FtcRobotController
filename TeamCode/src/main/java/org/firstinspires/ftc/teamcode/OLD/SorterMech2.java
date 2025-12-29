package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Sorting Test 2", group = "Test")
public class SorterMech2 extends LinearOpMode {

    private Servo sorter;
    private ColorSensor CS;

    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private List<String> targetOrder = new ArrayList<>();

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

            telemetry.addData("Detected Tag ID", tagId);

            switch (tagId) {
                case 1:
                    targetOrder.add("PURPLE");
                    targetOrder.add("GREEN");
                    targetOrder.add("PURPLE");
                    break;
                case 2:
                    targetOrder.add("GREEN");
                    targetOrder.add("GREEN");
                    targetOrder.add("PURPLE");
                    break;
                case 3:
                    targetOrder.add("PURPLE");
                    targetOrder.add("PURPLE");
                    targetOrder.add("GREEN");
                    break;
                default:
                    targetOrder.add("GREEN");
                    targetOrder.add("PURPLE");
                    targetOrder.add("GREEN");
                    telemetry.addLine("Unknown Tag ID → using default order");
            }
        } else {
            telemetry.addLine("No tag detected. Using default color order.");
            targetOrder.add("GREEN");
            targetOrder.add("PURPLE");
            targetOrder.add("GREEN");
        }

        telemetry.addData("Target Order:", targetOrder.toString());
        telemetry.update();

        sleep(1000);

        for (int i = 0; i < targetOrder.size() && opModeIsActive(); i++) {
            telemetry.addData("Expected Ball", i + 1);
            telemetry.addData("Target Color", targetOrder.get(i));
            telemetry.update();

            sleep(1000);

            String detectedColor = detectColor(CS);

            telemetry.addData("Detected Color", detectedColor);

            if (detectedColor.equals("PURPLE")) {
                sorter.setPosition(0.8);
                telemetry.addLine("→ PURPLE detected Moving servo RIGHT");
            } else if (detectedColor.equals("GREEN")) {
                sorter.setPosition(0.5);
                telemetry.addLine("→ GREEN detected Servo stays CENTER");
            } else {
                telemetry.addLine("→ UNKNOWN color  No action");
            }

            telemetry.update();
            sleep(1000);
        }

        telemetry.addLine("Sorting complete.");
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

