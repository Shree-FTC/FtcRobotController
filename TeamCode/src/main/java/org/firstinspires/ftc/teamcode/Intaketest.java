package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="MotifSortTest", group="Test")
public class Intaketest extends LinearOpMode {

    private Servo sortWheel;
    private Servo kicker1, kicker2, kicker3;
    private ColorSensor cs;

    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    private final double SLOT1_POS = 0.20;
    private final double SLOT2_POS = 0.50;
    private final double SLOT3_POS = 0.80;

    private final double REST_POS = 0.0;
    private final double FIRE_POS = 0.4;

    private String[] motifOrder = new String[3]; // X sets this

    @Override
    public void runOpMode() throws InterruptedException {

        // hardware init
        sortWheel = hardwareMap.get(Servo.class, "sortWheel");
        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");
        kicker3 = hardwareMap.get(Servo.class, "kicker3");
        cs = hardwareMap.get(ColorSensor.class, "cs");

        kicker1.setPosition(REST_POS);
        kicker2.setPosition(REST_POS);
        kicker3.setPosition(REST_POS);

        // vision init
        targetProcessor = new MyTargetProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                if (targetProcessor.targetDetected()) {
                    motifOrder = targetProcessor.getMotifOrder();
                    telemetry.addData("Motif Order", motifOrder[0] + ", " + motifOrder[1] + ", " + motifOrder[2]);
                } else {
                    telemetry.addData("Motif Order", "No target detected");
                }
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.a) {
                String[] colors = new String[3];

                // Scan each slot with color sensor
                moveToSlot(SLOT1_POS); colors[0] = readBallColor();
                moveToSlot(SLOT2_POS); colors[1] = readBallColor();
                moveToSlot(SLOT3_POS); colors[2] = readBallColor();

                // Fire in motif order
                for (String targetColor : motifOrder) {
                    for (int i = 0; i < 3; i++) {
                        if (colors[i].equals(targetColor)) {
                            switch (i) {
                                case 0: moveToSlot(SLOT1_POS); fire(kicker1); break;
                                case 1: moveToSlot(SLOT2_POS); fire(kicker2); break;
                                case 2: moveToSlot(SLOT3_POS); fire(kicker3); break;
                            }
                            colors[i] = "done"; // prevent double firing
                            break;
                        }
                    }
                }

                telemetry.addData("Action", "Completed shooting in motif order");
                telemetry.update();
                sleep(500); // debounce
            }
        }
    }

    private String readBallColor() {
        return (cs.green() > cs.red()) ? "green" : "purple";
    }

    private void moveToSlot(double pos) throws InterruptedException {
        sortWheel.setPosition(pos);
        sleep(300);
    }

    private void fire(Servo kicker) throws InterruptedException {
        kicker.setPosition(FIRE_POS);
        sleep(300);
        kicker.setPosition(REST_POS);
        sleep(200);
    }
}

