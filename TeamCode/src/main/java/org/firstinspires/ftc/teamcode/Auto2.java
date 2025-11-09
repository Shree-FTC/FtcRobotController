package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto w inches", group = "Autonomous")
public class Auto2 extends LinearOpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private DcMotor par, perp;

    static final double COUNTS_PER_INCH = 8192.0 / (Math.PI * 2.0);
    static final double ROBOT_TRACK_WIDTH = 16.0;
    static final double STRAFE_CORRECTION = 1.1;

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotor.class, "leftFront");
        frontright = hardwareMap.get(DcMotor.class, "rightFront");
        backleft = hardwareMap.get(DcMotor.class, "leftBack");
        backright = hardwareMap.get(DcMotor.class, "rightBack");

        par = hardwareMap.get(DcMotor.class, "par");
        perp = hardwareMap.get(DcMotor.class, "perp");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        resetOdometry();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        driveForwardInches(0.4, 24);
        strafeInches(0.4, 12, true);
        turnDegrees(0.3, 90);
    }

    public void driveForwardInches(double power, double inches) {
        int targetTicks = (int)(inches * COUNTS_PER_INCH);
        int startPar = par.getCurrentPosition();

        while (opModeIsActive()) {
            int deltaPar = par.getCurrentPosition() - startPar;
            if (Math.abs(deltaPar) >= Math.abs(targetTicks)) break;
            double direction = Math.signum(inches);
            setDrivePower(power * direction, power * direction);
        }

        Stop();
    }

    public void strafeInches(double power, double inches, boolean right) {
        int direction = right ? 1 : -1;
        int targetTicks = (int)(inches * COUNTS_PER_INCH * STRAFE_CORRECTION);
        int startPerp = perp.getCurrentPosition();

        while (opModeIsActive()) {
            int delta = perp.getCurrentPosition() - startPerp;
            if (Math.abs(delta) >= Math.abs(targetTicks)) break;
            setStrafePower(power * direction);
        }

        Stop();
    }

    public void turnDegrees(double power, double degrees) {
        double turnCircumference = ROBOT_TRACK_WIDTH * Math.PI;
        double turnDistance = (turnCircumference * (degrees / 360.0));
        int targetTicks = (int)(turnDistance * COUNTS_PER_INCH);
        int startPar = par.getCurrentPosition();

        while (opModeIsActive()) {
            int delta = Math.abs(par.getCurrentPosition() - startPar);
            if (delta >= Math.abs(targetTicks)) break;
            double direction = Math.signum(degrees);
            setDrivePower(power * direction, -power * direction);
        }

        Stop();
    }

    private void setDrivePower(double left, double right) {
        frontleft.setPower(left);
        backleft.setPower(left);
        frontright.setPower(right);
        backright.setPower(right);
    }

    private void setStrafePower(double power) {
        frontleft.setPower(power);
        backleft.setPower(-power);
        frontright.setPower(-power);
        backright.setPower(power);
    }

    private void Stop() {
        setDrivePower(0, 0);
        setStrafePower(0);
    }

    private void resetOdometry() {
        par.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
