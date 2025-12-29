package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@TeleOp(name = "AprilTag Localization Test 2 ", group = "Test")
public class Apriltaglocalizationtest2 extends LinearOpMode {

    private VisionPortal VP;
    private AprilTagProcessor ATP;

    private double robotX = 0;
    private double robotY = 0;
    private double robotZ = 0;
    private double robotYaw = 0;
    private double robotPitch = 0;
    private double robotRoll = 0;
    private double robotBearing = 0;
    private double robotRange = 0;
    private double robotElevation = 0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing camera...");
        telemetry.update();

        // ✅ Use AprilTagGameDatabase for Into the Deep field tags
        ATP = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .build();

        VP = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(ATP)
                .build();

        telemetry.addLine("Camera ready.");
        telemetry.addLine("Point camera at FTC field AprilTags.");
        telemetry.addLine("Press START to begin test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = ATP.getDetections();

            telemetry.addData("Tags Detected", detections.size());
            telemetry.addLine("");

            if (!detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine("===================");
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addLine("===================");
                    telemetry.addLine("");

                    if (detection.ftcPose != null) {
                        robotX = detection.ftcPose.x;
                        robotY = detection.ftcPose.y;
                        robotZ = detection.ftcPose.z;
                        robotYaw = detection.ftcPose.yaw;
                        robotPitch = detection.ftcPose.pitch;
                        robotRoll = detection.ftcPose.roll;
                        robotBearing = detection.ftcPose.bearing;
                        robotRange = detection.ftcPose.range;
                        robotElevation = detection.ftcPose.elevation;

                        telemetry.addLine("LOCALIZATION DATA:");
                        telemetry.addData("  X Position", "%.2f in", robotX);
                        telemetry.addData("  Y Position", "%.2f in", robotY);
                        telemetry.addData("  Z Position", "%.2f in", robotZ);
                        telemetry.addLine("");

                        telemetry.addLine("ORIENTATION:");
                        telemetry.addData("  Yaw", "%.2f°", robotYaw);
                        telemetry.addData("  Pitch", "%.2f°", robotPitch);
                        telemetry.addData("  Roll", "%.2f°", robotRoll);
                        telemetry.addLine("");

                        telemetry.addLine("RELATIVE TO TAG:");
                        telemetry.addData("  Bearing", "%.2f°", robotBearing);
                        telemetry.addData("  Range", "%.2f in", robotRange);
                        telemetry.addData("  Elevation", "%.2f°", robotElevation);
                        telemetry.addLine("");

                        telemetry.addLine("✓ LOCALIZATION ACTIVE");
                        telemetry.addData("  Distance from tag", "%.2f in", robotRange);
                        telemetry.addData("  Angle to tag", "%.2f°", robotBearing);
                    } else {
                        telemetry.addLine("✗ NO POSE DATA AVAILABLE");
                        telemetry.addLine("Tag detected but no position");
                    }

                    if (detection.metadata != null) {
                        telemetry.addLine("");
                        telemetry.addLine("TAG INFO:");
                        telemetry.addData("  Name", detection.metadata.name);
                        telemetry.addData("  Size", "%.2f in", detection.metadata.tagsize);
                    }

                    telemetry.addLine("");
                }
            } else {
                telemetry.addLine("✗ NO APRILTAGS DETECTED");
                telemetry.addLine("");
                telemetry.addLine("Make sure:");
                telemetry.addLine("  - Camera is pointing at tag");
                telemetry.addLine("  - Tag is well lit");
                telemetry.addLine("  - Tag is not too far");
                telemetry.addLine("  - Tag is in focus");
                telemetry.addLine("");
                telemetry.addData("Last Known X", "%.2f", robotX);
                telemetry.addData("Last Known Y", "%.2f", robotY);
                telemetry.addData("Last Known Bearing", "%.2f", robotBearing);
            }

            telemetry.update();
            sleep(100);
        }

        VP.close();
    }
}