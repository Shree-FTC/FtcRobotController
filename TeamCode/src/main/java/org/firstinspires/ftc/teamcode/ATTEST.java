package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ATTEST",group = "Autonomous")
   public class ATTEST extends LinearOpMode {
     protected DcMotor frontleft;
     protected DcMotor frontright;
     protected DcMotor backleft;
     protected DcMotor backright;
     protected DcMotor ascentMotor;
     protected DcMotor slideMotor;
     protected CRServo shoulderServo;
     protected CRServo elbowServo;
     protected CRServo servo0;
     @Override
     public void runOpMode() throws InterruptedException {
         frontleft = hardwareMap.get(DcMotor.class, "frontLeft");
         frontright = hardwareMap.get(DcMotor.class, "frontRight");
         backleft = hardwareMap.get(DcMotor.class, "backLeft");
         backright = hardwareMap.get(DcMotor.class, "backRight");
         ascentMotor = hardwareMap.get(DcMotor.class, "ascentmotor");
         slideMotor = hardwareMap.get(DcMotor.class, "slidemotor");
         shoulderServo = hardwareMap.get(CRServo.class, "shoulderServo");
         elbowServo = hardwareMap.get(CRServo.class, "elbowServo");
         servo0 = hardwareMap.get(CRServo.class, "servo0");
         strafeRight(0.5,2);
            }
    public void moveforward (double power, int time){
        frontleft.setPower(power);
        frontright.setPower(power);
        backleft.setPower(power);
        backright.setPower(power);
    }
    public void turnright (double power, int time) {
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
    public void turnleft (double power, int time){
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
    public void movebackward (double power, int time){
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
    public void GoToY() {
        final double SHOULDER_OUT = 0.15;      // 38/360 degrees
        final double SHOULDER_EXTENDED = 0.80; // 263/360 degrees
        final double ELBOW_EXTENDED = 0.50;
        final double ELBOW_OUT = 0.52;        // 90/360 degrees
        final double ORIGIN = 0;              // 0 degrees
        final double ELBOW_ORIGIN = 1;      // Variables to track current positions
        double currentShoulderPos = ORIGIN;
        double currentElbowPos = ELBOW_ORIGIN;
        String currentArmMode = "Initial"; //Grab, Drop

        shoulderServo.setPower(ORIGIN);
        elbowServo.setPower(ELBOW_ORIGIN);
        currentShoulderPos = ORIGIN;
        currentElbowPos = ELBOW_ORIGIN;
        currentArmMode = "Initial";
    }
    public void GoToA() {
        final double SHOULDER_OUT = 0.15;      // 38/360 degrees
        final double SHOULDER_EXTENDED = 0.80; // 263/360 degrees
        final double ELBOW_EXTENDED = 0.50;
        final double ELBOW_OUT = 0.52;        // 90/360 degrees
        final double ORIGIN = 0;              // 0 degrees
        final double ELBOW_ORIGIN = 1;      // Variables to track current positions
        double currentShoulderPos = ORIGIN;
        double currentElbowPos = ELBOW_ORIGIN;
        String currentArmMode = "Initial"; //Grab, Drop
        shoulderServo.setPower(SHOULDER_OUT);
        currentShoulderPos = SHOULDER_OUT;
        sleep(1000);
        elbowServo.setPower(ELBOW_OUT);
        currentElbowPos = ELBOW_OUT;
        currentArmMode = "Grab";
    }
    public void Stop (){
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }
    public void extend (double power){
        slideMotor.setPower(power);
    }
    public void deexstend(double power){
        slideMotor.setPower(-power);
        sleep(1000);
        slideMotor.setPower(0);
    }
    public void grab(double power){
        servo0.setPower(power);
    }
    public void drop(double power){
        servo0.setPower(-power);
        sleep(1000);
        servo0.setPower(0);
    }
    public void strafeLeft(double power,int time){
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
    public void strafeRight(double power, int time){
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