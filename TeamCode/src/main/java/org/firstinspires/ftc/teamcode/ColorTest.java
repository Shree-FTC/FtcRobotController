package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor", group = "Sensor")
public class ColorTest extends LinearOpMode {

    private ColorSensor CS;

    @Override
    public void runOpMode() {

        CS = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {

            int red = CS.red();
            int green = CS.green();
            int blue = CS.blue();
            if (green > blue && green > red && green>350) {
                telemetry.addLine("Green object detected!");
            } else {
                telemetry.addLine("Not Green.");
            }

            // Show color values
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();


        }
    }
}

