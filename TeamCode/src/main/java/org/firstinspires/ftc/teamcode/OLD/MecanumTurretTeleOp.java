package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="MecanumTeleOp", group="Main")
public class MecanumTurretTeleOp extends LinearOpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor turret;
    DcMotor shooter;
    Servo roll1, roll2, roll3;


    boolean shooterOn = false;
    boolean shooterButtonPressed = false;

    boolean intakeOn = false;
    boolean intakeButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        turret = hardwareMap.dcMotor.get("turret");
        shooter = hardwareMap.dcMotor.get("shooter");

        roll1 = hardwareMap.get(Servo.class, "roll1");
        roll2 = hardwareMap.get(Servo.class, "roll2");
        roll3 = hardwareMap.get(Servo.class, "roll3");

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);


            if (gamepad2.a) {
                turret.setPower(0.4);
            } else {
                turret.setPower(0);
            }


            if (gamepad2.b && !shooterButtonPressed) {
                shooterOn = !shooterOn;
                shooterButtonPressed = true;
            }
            if (!gamepad2.b) shooterButtonPressed = false;

            shooter.setPower(shooterOn ? 1.0 : 0.0);

            if (gamepad2.x && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.x) intakeButtonPressed = false;

            double rollPower = intakeOn ? 1.0 : 0.0;
            roll1.setPosition(rollPower);
            roll2.setPosition(rollPower);
            roll3.setPosition(rollPower);


            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
