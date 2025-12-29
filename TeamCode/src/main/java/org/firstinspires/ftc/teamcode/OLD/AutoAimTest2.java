package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayDeque;

@TeleOp(name="AutoAimTest2", group="Test")
public class AutoAimTest2 extends LinearOpMode {
//Values Set Up
    private Servo turretYaw;
    private Servo turretPitch;
    private HuskyLens huskyLens;

    private static final int IMAGE_WIDTH = 320;
    private static final int IMAGE_HEIGHT = 240;

    private static final double CAMERA_HFOV_DEG = 61.0;
    private static final double CAMERA_VFOV_DEG = 47.0;

    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_RANGE_DEG = CAMERA_HFOV_DEG;

    private static final double PITCH_MIN = 0.0;
    private static final double PITCH_MAX = 1.0;
    private static final double PITCH_CENTER = 0.5;
    private static final double PITCH_RANGE_DEG = CAMERA_VFOV_DEG;

    private static final double DEADBAND_DEG = 1.5;
    private static final int SMOOTHING_SAMPLES = 5;

    private ArrayDeque<Double> yawBuffer = new ArrayDeque<>();
    private ArrayDeque<Double> pitchBuffer = new ArrayDeque<>();
    private double lastYawPos;
    private double lastPitchPos;

    @Override
    public void runOpMode() {
        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretPitch = hardwareMap.get(Servo.class, "turretPitch");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        //lastYawPos = YAW_CENTER;
       // lastPitchPos = PITCH_CENTER;
       // turretYaw.setPosition(lastYawPos);
        //turretPitch.setPosition(lastPitchPos);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks != null && blocks.length > 0) {
                    HuskyLens.Block tag = blocks[0];

                    double xOffset = tag.x - (IMAGE_WIDTH / 2.0);
                    double yawError = (xOffset / IMAGE_WIDTH) * CAMERA_HFOV_DEG;

                    double yOffset = (IMAGE_HEIGHT / 2.0) - tag.y;
                    double pitchError = (yOffset / IMAGE_HEIGHT) * CAMERA_VFOV_DEG;

                    yawBuffer.add(yawError);
                    if (yawBuffer.size() > SMOOTHING_SAMPLES)
                        yawBuffer.poll();

                    pitchBuffer.add(pitchError);
                    if (pitchBuffer.size() > SMOOTHING_SAMPLES)
                        pitchBuffer.poll();

                    double avgYaw = 0;
                    for (double a : yawBuffer) avgYaw += a;
                    avgYaw /= yawBuffer.size();

                    double avgPitch = 0;
                    for (double a : pitchBuffer) avgPitch += a;
                    avgPitch /= pitchBuffer.size();
                    //Turret Yaw moving to Best
                    if (Math.abs(avgYaw) > DEADBAND_DEG) {
                        double yawOffset = avgYaw / YAW_RANGE_DEG;
                        double newYawPos = lastYawPos + yawOffset;
                        newYawPos = Math.max(YAW_MIN, Math.min(YAW_MAX, newYawPos));
                        turretYaw.setPosition(newYawPos);
                        lastYawPos = newYawPos;
                    }
                    //Turret Pitch moving to Best
                    if (Math.abs(avgPitch) > DEADBAND_DEG) {
                        double pitchOffset = avgPitch / PITCH_RANGE_DEG;
                        double newPitchPos = lastPitchPos + pitchOffset;
                        newPitchPos = Math.max(PITCH_MIN, Math.min(PITCH_MAX, newPitchPos));
                        turretPitch.setPosition(newPitchPos);
                        lastPitchPos = newPitchPos;
                    }

                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Yaw Error", "%.2f°", avgYaw);
                    telemetry.addData("Pitch Error", "%.2f°", avgPitch);
                    telemetry.addData("Yaw Pos", "%.3f", lastYawPos);
                    telemetry.addData("Pitch Pos", "%.3f", lastPitchPos);

                } else {
                    yawBuffer.clear();
                    pitchBuffer.clear();
                    telemetry.addLine("NO TARGET");
                }
            }

            telemetry.update();
            sleep(40);
        }
    }
}