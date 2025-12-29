package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="TeleOp", group="Linear OpMode")
public class Tele extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private CRServo sorter;
    ColorSensor CS;

    @Override
    public void runOpMode() {


        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        sorter = hardwareMap.get(CRServo.class, "sorter");
        CS = hardwareMap.get(ColorSensor.class, "colorSensor");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sorter.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();



        waitForStart();

        while (opModeIsActive()) {

        }
    }
    private void forward(){
        while(gamepad1.dpad_up) {
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
        }
    }
    private void backward(){
        while(gamepad1.dpad_down) {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(-1);
        }
    }
    private void strafeleft(){
        while(gamepad1.left_bumper) {
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
    }
    private void straferight(){
        while(gamepad1.right_bumper){
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
    }
    private void right(){
        while(gamepad1.right_bumper){
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
    }
    private void left(){
        while(gamepad1.right_bumper){
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
    }
    private void turnright(){
        while(gamepad1.right_bumper){
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
    }
    private void sort(){
        if(gamepad1.a){
            int red = CS.red();
            int green = CS.green();
            int blue = CS.blue();
            if (green > blue && green > red) {
                telemetry.addLine("Ball is Green.");
                sorter.setPower(1);
                sleep(100);
                sorter.setPower(0);
            } else {
                telemetry.addLine("Ball is Purple.");
            }
        }
    }
}