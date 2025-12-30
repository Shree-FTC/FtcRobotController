package org.firstinspires.ftc.teamcode.Test_Folder;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Localization Test", group = "Test")
public class huskylocalizationtest extends LinearOpMode {

    private HuskyLens huskyLens;

    private double robotX = 0;
    private double robotY = 0;
    private double robotBearing = 0;
    private double robotDistance = 0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing HuskyLens...");
        telemetry.update();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("HuskyLens ready.");
        telemetry.addLine("Point camera at learned tags/objects.");
        telemetry.addLine("Press START to begin test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Objects Detected", blocks.length);
            telemetry.addLine("");

            if (blocks.length > 0) {
                for (HuskyLens.Block block : blocks) {
                    telemetry.addLine("===================");
                    telemetry.addData("Object ID", block.id);
                    telemetry.addLine("===================");
                    telemetry.addLine("");

                    int centerX = block.x;
                    int centerY = block.y;
                    int width = block.width;
                    int height = block.height;

                    double pixelOffset = centerX - 160;
                    robotBearing = (pixelOffset / 160.0) * 30.0;

                    robotDistance = (100.0 * 50.0) / width;

                    robotX = robotDistance * Math.sin(Math.toRadians(robotBearing));
                    robotY = robotDistance * Math.cos(Math.toRadians(robotBearing));

                    telemetry.addLine("DETECTION DATA:");
                    telemetry.addData("  Center X", centerX);
                    telemetry.addData("  Center Y", centerY);
                    telemetry.addData("  Width", width);
                    telemetry.addData("  Height", height);
                    telemetry.addLine("");

                    telemetry.addLine("ESTIMATED POSITION:");
                    telemetry.addData("  Relative X", "%.2f in", robotX);
                    telemetry.addData("  Relative Y", "%.2f in", robotY);
                    telemetry.addData("  Bearing", "%.2f°", robotBearing);
                    telemetry.addData("  Distance", "%.2f in", robotDistance);
                    telemetry.addLine("");

                    telemetry.addLine("✓ OBJECT DETECTED");
                    telemetry.addData("  Angle from center", "%.1f°", robotBearing);
                    telemetry.addData("  Est. distance", "%.1f in", robotDistance);
                    telemetry.addLine("");
                }
            } else {
                telemetry.addLine("✗ NO OBJECTS DETECTED");
                telemetry.addLine("");
                telemetry.addLine("Make sure:");
                telemetry.addLine("  - HuskyLens is in Tag Recognition mode");
                telemetry.addLine("  - Tags/objects are learned in HuskyLens");
                telemetry.addLine("  - Object is in view");
                telemetry.addLine("  - Good lighting conditions");
                telemetry.addLine("");
                telemetry.addData("Last Known X", "%.2f", robotX);
                telemetry.addData("Last Known Y", "%.2f", robotY);
                telemetry.addData("Last Known Bearing", "%.2f°", robotBearing);
            }

            telemetry.update();
            sleep(100);
        }
    }
}