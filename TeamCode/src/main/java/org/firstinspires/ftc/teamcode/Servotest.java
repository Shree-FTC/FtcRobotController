package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest", group="Example")
public class Servotest extends LinearOpMode {

    private Servo turret;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(0.5);

        telemetry.addLine("Ready. Press A (0) or B (1).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a) {
                turret.setPosition(0);
                telemetry.addData("Servo Pos", "0");
            }

            if (gamepad2.b) {
                turret.setPosition(1);
                telemetry.addData("Servo Pos", "1");
            }

            telemetry.update();
        }
    }
}

