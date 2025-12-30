package org.firstinspires.ftc.teamcode.OLD;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="nanoteleop",group = "TeleOp")
public class nanoteleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        ascentMotor = hardwareMap.get(DcMotor.class, "ascentmotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slidemotor");
        shoulderServo = hardwareMap.get(CRServo.class, "shoulderServo");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");
        servo0 = hardwareMap.get(CRServo.class, "servo0");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grab();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double drive = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_y;
            //turn = turn / 1.5;
            double leftPower = Range.clip(drive + turn, -0.5, 0.5);
            double rightPower = Range.clip(drive - turn, -0.5, 0.5);

            frontleft.setPower(leftPower);
            frontright.setPower(rightPower);
            backleft.setPower(leftPower);
            backright.setPower(rightPower);
            // Sliding arm control using D-pad
            if (gamepad1.dpad_up) {
                extend();  // Slide arm out
            } else if (gamepad1.dpad_down) {
                deexstend(); // Slide arm in
            } else {
                slideMotor.setPower(0); // Stop sliding motor when no input
            }

            if (gamepad1.left_bumper) {
                drop();
            } else if (gamepad1.right_bumper) {
                grab();
            }
            if (gamepad1.dpad_left) {
                strafeLeft();
            } else if (gamepad1.dpad_right) {
                strafeRight();
            }
            if(gamepad1.left_stick_button){
                GoToA();
            }else if(gamepad1.right_stick_button){
                GoToY();
            }
            if(gamepad1.a){
                ascent();
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f), slide (%.2f)", leftPower, rightPower, slideMotor.getPower());
            telemetry.update();

        }

    }

    public void moveforward() {
        frontleft.setPower(1);
        frontright.setPower(1);
        backleft.setPower(1);
        backright.setPower(1);
    }

    public void turnright() {
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(1);
        backright.setPower(-1);
    }

    public void turnleft() {
        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(-1);
        backright.setPower(1);
    }

    public void movebackward() {
        frontleft.setPower(-1);
        frontright.setPower(-1);
        backleft.setPower(-1);
        backright.setPower(-1);
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

    public void Stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

    public void extend() {
        slideMotor.setPower(0.5);
    }

    public void deexstend() {
        slideMotor.setPower(-0.5);
    }

    public void grab() {
        servo0.setPower(1);
    }

    public void drop() {
        servo0.setPower(-1);
    }

    public void strafeLeft() {
        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(1);
        backright.setPower(-1);

    }

    public void strafeRight() {
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(-1);
        backright.setPower(1);

    }

    public void ascent() {
        ascentMotor.setPower(1);
    }
}