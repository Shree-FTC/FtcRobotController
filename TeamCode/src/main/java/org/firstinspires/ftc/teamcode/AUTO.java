package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DriveAuto",group = "Autonomous")
public class AUTO extends LinearOpMode {
    protected DcMotor frontleft;
    protected DcMotor frontright;
    protected DcMotor backleft;
    protected DcMotor backright;

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        waitForStart();
        moveforward(0.5,1000);
        turnright(0.5, 1000);
        turnleft(0.5,1000);
        strafeRight(0.5, 300);
        strafeLeft(0.5, 300);
        movebackword(0.5, 1000);
        Stop();
    }
    public void turnright (double power, int time){
        frontleft.setPower(power);
        frontright.setPower(power);
        backleft.setPower(power);
        backright.setPower(power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void strafeRight (double power, int time) {
        frontleft.setPower(power);
        frontright.setPower(-power);
        backleft.setPower(power);
        backright.setPower(-power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void strafeLeft (double power, int time){
        frontleft.setPower(-power);
        frontright.setPower(power);
        backleft.setPower(-power);
        backright.setPower(power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

    }
    public void turnleft (double power, int time){
        frontright.setPower(-power);
        backleft.setPower(-power);
        backright.setPower(-power);
        frontleft.setPower(-power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void Stop (){
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }
    public void movebackword (double power,int time){
        frontleft.setPower(-power);
        frontright.setPower(power);
        backleft.setPower(power);
        backright.setPower(-power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);


    }
    public void moveforward(double power, int time){
        frontleft.setPower(power);
        frontright.setPower(-power);
        backleft.setPower(-power);
        backright.setPower(power);
        sleep(time);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}
