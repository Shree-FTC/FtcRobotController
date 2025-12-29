package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="AprilTag Autonomous2)", group="Autonomous")
public class Auto4 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private Servo sorter1;
    private Servo sorter2;

    private VisionPortal VP;
    private AprilTagProcessor ATP;

    // Servo positions (mirrored)
    private static final double PURPLE_POSITION_1 = 0.2;
    private static final double GREEN_POSITION_1  = 0.8;
    private static final double PURPLE_POSITION_2 = 0.8;
    private static final double GREEN_POSITION_2  = 0.2;

    private static final double COUNTS_PER_INCH = 50;
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.3;

    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing autonomous...");
        telemetry.update();

        initializeHardware();

        telemetry.addLine("Scanning for AprilTag...");
        telemetry.update();

        AprilTagDetection targetTag = scanForAprilTag();

        if (targetTag != null) {
            telemetry.addData("Target Tag", targetTag.id);
            telemetry.addData("Position X", "%.2f", targetTag.ftcPose.x);
            telemetry.addData("Position Y", "%.2f", targetTag.ftcPose.y);
            telemetry.addData("Bearing", "%.2f", targetTag.ftcPose.bearing);
            telemetry.update();
        } else {
            telemetry.addLine("No AprilTag detected - using default path");
            telemetry.update();
        }

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runAutonomousSequence(targetTag);
        }
    }

    private void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        sorter1 = hardwareMap.get(Servo.class, "sorterServo1");
        sorter2 = hardwareMap.get(Servo.class, "sorterServo2");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ATP = AprilTagProcessor.easyCreateWithDefaults();
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                ATP
        );

        setSorterNeutral();
    }

    private AprilTagDetection scanForAprilTag() {
        sleep(500);

        long startTime = System.currentTimeMillis();
        AprilTagDetection detectedTag = null;

        while (System.currentTimeMillis() - startTime < 3000 && !isStarted()) {
            List<AprilTagDetection> detections = ATP.getDetections();

            if (!detections.isEmpty()) {
                detectedTag = detections.get(0);
                telemetry.addData("Tag Found", detectedTag.id);
                telemetry.update();
                break;
            }

            sleep(100);
        }

        return detectedTag;
    }

    private void runAutonomousSequence(AprilTagDetection tag) {
        telemetry.addLine("Starting autonomous sequence");
        telemetry.update();

        if (tag != null) {
            updateLocalizationFromTag(tag);
            navigateToScoringPosition(tag);
        } else {
            driveForward(24);
        }

        scoreBalls();

        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }

    private void updateLocalizationFromTag(AprilTagDetection tag) {
        robotX = tag.ftcPose.x;
        robotY = tag.ftcPose.y;
        robotHeading = tag.ftcPose.bearing;

        telemetry.addData("Localization Updated", "");
        telemetry.addData("X", "%.2f", robotX);
        telemetry.addData("Y", "%.2f", robotY);
        telemetry.addData("Heading", "%.2f", robotHeading);
        telemetry.update();
    }

    private void navigateToScoringPosition(AprilTagDetection tag) {
        telemetry.addLine("Navigating to scoring position");
        telemetry.update();

        double distanceToTag = Math.sqrt(tag.ftcPose.x * tag.ftcPose.x + tag.ftcPose.y * tag.ftcPose.y);
        double angleToTag = Math.toDegrees(Math.atan2(tag.ftcPose.y, tag.ftcPose.x));

        turnToAngle(angleToTag);
        driveForward(distanceToTag - 12);

        AprilTagDetection updatedTag = scanForAprilTag();
        if (updatedTag != null) {
            updateLocalizationFromTag(updatedTag);

            double correctionAngle = updatedTag.ftcPose.bearing;
            if (Math.abs(correctionAngle) > 5) {
                turnToAngle(correctionAngle);
            }
        }
    }

    private void scoreBalls() {
        telemetry.addLine("Scoring purple ball");
        telemetry.update();
        setSorterPurple();
        sleep(1000);

        telemetry.addLine("Scoring green ball");
        telemetry.update();
        setSorterGreen();
        sleep(1000);

        telemetry.addLine("Scoring purple ball");
        telemetry.update();
        setSorterPurple();
        sleep(1000);
    }

    private void driveForward(double inches) {
        int targetPosition = (int)(inches * COUNTS_PER_INCH);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(targetPosition);
        rightFront.setTargetPosition(targetPosition);
        leftBack.setTargetPosition(targetPosition);
        rightBack.setTargetPosition(targetPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);

        while (opModeIsActive() && leftFront.isBusy()) {
            telemetry.addData("Driving", "%.2f inches", inches);
            telemetry.addData("Position", leftFront.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    private void turnToAngle(double degrees) {
        int targetPosition = (int)(degrees * COUNTS_PER_INCH * 0.5);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(targetPosition);
        rightFront.setTargetPosition(-targetPosition);
        leftBack.setTargetPosition(targetPosition);
        rightBack.setTargetPosition(-targetPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(TURN_SPEED);
        rightFront.setPower(TURN_SPEED);
        leftBack.setPower(TURN_SPEED);
        rightBack.setPower(TURN_SPEED);

        while (opModeIsActive() && leftFront.isBusy()) {
            telemetry.addData("Turning", "%.2f degrees", degrees);
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void setSorterPurple() {
        sorter1.setPosition(PURPLE_POSITION_1);
        sorter2.setPosition(PURPLE_POSITION_2);
    }

    private void setSorterGreen() {
        sorter1.setPosition(GREEN_POSITION_1);
        sorter2.setPosition(GREEN_POSITION_2);
    }

    private void setSorterNeutral() {
        sorter1.setPosition(0.5);
        sorter2.setPosition(0.5);
    }
}
