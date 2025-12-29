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
import java.util.Locale;

@Autonomous(name = "AprilTag Full Field Auto Shoot ", group = "Auto")
public class ShootandDrive extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor shooterMotor;
    private Servo feeder;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static class Pose2d {
        public double x, y, heading; // heading in degrees
        public Pose2d(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    private Pose2d robotPose = new Pose2d(0, 0, 0);

    private final Pose2d[] SHOOT_SPOTS = {
            new Pose2d(1.2, 2.0, -90),
            new Pose2d(0.6, 1.5, -90),
            new Pose2d(0.3, 1.0, -90)
    };

    private static final double KP_DRIVE = 0.02;
    private static final double KP_TURN = 0.015;
    private static final double POSITION_TOL = 0.05;
    private static final double ANGLE_TOL = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        initVision();

        telemetry.addLine("Ready to scan for tags...");
        telemetry.update();

        waitForStart();

        Pose2d detectedPose = getRobotPoseFromTags();
        if (detectedPose == null) {
            telemetry.addLine("No tags detected, aborting!");
            telemetry.update();
            sleep(2000);
            return;
        }
        robotPose = detectedPose;
        telemetry.addData("Robot Pose", formatPose(robotPose));
        telemetry.update();
        sleep(1000);

        Pose2d bestSpot = chooseBestSpot(robotPose);
        telemetry.addData("Chosen Spot", formatPose(bestSpot));
        telemetry.update();
        sleep(1000);

        driveToTarget(bestSpot);
        aimAtTarget(bestSpot.heading);

        shootSequence();

        stopAllMotors();
        telemetry.addLine("Sequence complete!");
        telemetry.update();
        sleep(3000);
    }

    private void initHardware() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        feeder       = hardwareMap.get(Servo.class, "feeder");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawTagOutline(true).build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private Pose2d getRobotPoseFromTags() {
        long startTime = System.currentTimeMillis();
        long timeout = 3000; // 3 seconds
        Pose2d result = null;

        while (opModeIsActive() && System.currentTimeMillis() - startTime < timeout && result == null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection det : detections) {
                if (det.metadata != null) {
                    // Use the detection's own pose relative to camera
                    // det.ftcPose contains the pose of the tag relative to the robot
                    double tagX = det.ftcPose.x;
                    double tagY = det.ftcPose.y;
                    double tagHeading = det.ftcPose.bearing;

                    result = new Pose2d(tagX, tagY, tagHeading);

                    telemetry.addData("Located Tag ID", det.id);
                    telemetry.addData("Robot Pose", formatPose(result));
                    telemetry.update();
                    break;
                }
            }

            if (result == null) {
                sleep(50);
            }
        }
        return result;
    }

    private Pose2d chooseBestSpot(Pose2d current) {
        Pose2d best = SHOOT_SPOTS[0];
        double bestDist = Double.MAX_VALUE;
        for (Pose2d s : SHOOT_SPOTS) {
            double dist = Math.hypot(s.x - current.x, s.y - current.y);
            if (dist < bestDist) {
                best = s;
                bestDist = dist;
            }
        }
        return best;
    }

    private void driveToTarget(Pose2d target) throws InterruptedException {
        long timeout = System.currentTimeMillis() + 7000;
        while (opModeIsActive() && System.currentTimeMillis() < timeout) {

            Pose2d currentPose = getRobotPoseFromTags();
            if (currentPose != null) robotPose = currentPose;

            double dx = target.x - robotPose.x;
            double dy = target.y - robotPose.y;
            double dist = Math.hypot(dx, dy);

            if (dist < POSITION_TOL) break;

            double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
            double angleError = normalizeAngle(targetAngle - robotPose.heading);

            double drivePower = clamp(dist * KP_DRIVE, -0.6, 0.6);
            double turnPower  = clamp(angleError * KP_TURN, -0.5, 0.5);

            setDrivePower(drivePower - turnPower, drivePower + turnPower);

            telemetry.addData("Driving to", formatPose(target));
            telemetry.addData("Current", formatPose(robotPose));
            telemetry.update();
            sleep(40);
        }
        stopAllMotors();
    }

    private void aimAtTarget(double targetHeading) throws InterruptedException {
        long timeout = System.currentTimeMillis() + 4000;
        while (opModeIsActive() && System.currentTimeMillis() < timeout) {
            Pose2d currentPose = getRobotPoseFromTags();
            if (currentPose != null) robotPose = currentPose;

            double error = normalizeAngle(targetHeading - robotPose.heading);
            if (Math.abs(error) < ANGLE_TOL) break;

            // P controller for turning
            double turnPower = clamp(error * KP_TURN, -0.5, 0.5);
            if (Math.abs(turnPower) < 0.1) {
                turnPower = Math.signum(turnPower) * 0.1; // Overcome friction
            }
            setDrivePower(-turnPower, turnPower);

            telemetry.addData("Aiming error", "%.1f deg", error);
            telemetry.update();
            sleep(40);
        }
        stopAllMotors();
    }

    private void shootSequence() throws InterruptedException {
        telemetry.addLine("Shooting...");
        telemetry.update();
        shooterMotor.setPower(1.0);
        sleep(1000); // spin-up
        feeder.setPosition(1.0);
        sleep(400);
        feeder.setPosition(0.0);
        shooterMotor.setPower(0);
    }

    private void setDrivePower(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    private void stopAllMotors() {
        setDrivePower(0, 0);
        shooterMotor.setPower(0);
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private String formatPose(Pose2d p) {
        return String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.1f°", p.x, p.y, p.heading);
    }
}