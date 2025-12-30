/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="NanoKnights: TeleOpMode", group="Linear OpMode")
public class NanoKnightMecanum extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private DcMotor slideMotor = null;

    private CRServo leftServo = null;
    private CRServo rightServo = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //update the string values to match the motor names in the robot configuration
        leftDrive  = hardwareMap.get(DcMotor.class, "leftmotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightmotor");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slidermotor");
        leftServo = hardwareMap.get(CRServo.class, "servo0");
        rightServo = hardwareMap.get(CRServo.class, "servo1");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        closeClaw();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double drive = gamepad1.right_stick_x;
            double turn  = gamepad1.left_stick_y;
            //turn = turn / 1.5;
            double leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            double rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;
            double armPower = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.5; // Right trigger moves arm up, left trigger moves arm down

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            armMotor.setPower(armPower);
            // Sliding arm control using D-pad
            if (gamepad1.dpad_up) {
                slideMotor.setPower(-0.5);  // Slide arm out
            } else if (gamepad1.dpad_down) {
                slideMotor.setPower(0.5); // Slide arm in
            } else {
                slideMotor.setPower(0); // Stop sliding motor when no input
            }

            if (gamepad1.left_bumper){
                openClaw();
            }else if (gamepad1.right_bumper){
                closeClaw();
            }
            if (gamepad1.dpad_left){
                openClaw();
            }else if (gamepad1.dpad_right){
                closeClaw();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f), slide (%.2f)", leftPower, rightPower, armPower, slideMotor.getPower());
            telemetry.update();
        }
    }

    private void openClaw(){
     leftServo.setPower(-0.5);
     rightServo.setPower(-0.5);
     telemetry.addData("Claw", "Opened (%.2f) (%.2f)", leftServo.getPower(), rightServo.getPower());
    }

    private void closeClaw(){
        leftServo.setPower(0.5);
        rightServo.setPower(0.5);
        telemetry.addData("Claw", "Closed (%.2f) (%.2f)", leftServo.getPower(), rightServo.getPower());
    }
}