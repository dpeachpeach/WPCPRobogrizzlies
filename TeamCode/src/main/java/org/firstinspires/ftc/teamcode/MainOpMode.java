package org.firstinspires.ftc.teamcode;
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MainOpMode", group="Linear Opmode")
public class MainOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // Motor on the front left
    public DcMotor leftDrive1 = null;
    // Motor on the front right
    public DcMotor rightDrive1 = null;
    // Motor on the back left
    public DcMotor leftDrive2 = null;
    // Motor on the back right
    public DcMotor rightDrive2 = null;
    // Conveyor belt
    public DcMotor conveyorBelt = null;
    // Left shooter motor (ultraplanetary)
    public DcMotor leftShooter = null;
    // Right shooter motor (ultraplanetary)
    public DcMotor rightShooter = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // DAVID PETRE: Initialize hardware vars, see note below.
        // Be advised, until robot is properly configured, the devicenames are PLACEHOLDERS.
        // We need to replace them with what we eventually write down when we config

        leftDrive1  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_back_drive");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor");
        leftShooter = hardwareMap.get(DcMotor.class, "left_shooter");
        rightShooter = hardwareMap.get(DcMotor.class, "right_shooter");

        // runs the moment robot is initialized
        waitForStart();
        runtime.reset();


        // runs after driver presses play
        while (opModeIsActive()) {
            // telemetry variable setup. Eventual setup for encoders. (DP)
            double leftPowerFront;
            double rightPowerFront;
            double leftPowerBack;
            double rightPowerBack;
            double conveyorPower;
            double rightShooterPower = 0;
            double leftShooterPower = 0;


            // POV Mode uses left stick to go forward and to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double lateral = -gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turning = -gamepad1.right_stick_x;

            leftPowerFront = Range.clip(turning + vertical - lateral, -1.0, 1.0);
            leftPowerBack = Range.clip(turning + vertical + lateral, -1.0, 1.0);
            rightPowerFront = Range.clip(turning - vertical - lateral, -1.0, 1.0);
            rightPowerBack = Range.clip(turning - vertical + lateral, -1.0, 1.0);

            double conveyorInput = gamepad1.right_stick_y;
            conveyorPower = Range.clip(conveyorInput, -1.0, 1.0);

            // inefficient functions for simplification of code + to get shooters to work (Keep this for built in redundancy please)
            if (gamepad1.right_bumper){
                rightShooterPower = 1;
                leftShooterPower = 1;
            }
            if (!gamepad1.right_bumper){
                rightShooterPower = 0;
                leftShooterPower = 0;
            }

            // I have to talk to Fay on how they want to engineer the turn / drive difference here.
            // Whether we'll do a  4-wheel drive style thing or just have one wheel for a turn

            // Send calculated power to wheels
            leftDrive1.setPower(leftPowerFront);
            rightDrive1.setPower(rightPowerFront);
            leftDrive2.setPower(leftPowerBack);
            rightDrive2.setPower(rightPowerBack);
            conveyorBelt.setPower(conveyorPower);
            rightShooter.setPower(rightShooterPower);
            leftShooter.setPower(leftShooterPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData(
                    "Motors",
                    "leftfront (%.2f), rightfront (%.2f), leftback (%.2f), rightback (%.2f), conveyor (%.2f)",
                    leftPowerFront,
                    rightPowerFront,
                    leftPowerBack,
                    rightPowerBack,
                    conveyorPower
            );
            telemetry.update();
        }
    }
}
