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
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;


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

@TeleOp(name="AutonomousOpMode", group="Linear Opmode")
public class AutonomousOpMode extends LinearOpMode {

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

        ArrayList<AutonomousStep> steps = new ArrayList<>();
        addAutonomousSteps(steps);

        // runs after driver presses play
        for (AutonomousStep step : steps) {
            if (!opModeIsActive())
                break;

            step.run(this);
        }
    }

    void setAllDriveMotorPower(double power) {
        leftDrive1.setPower(power);
        leftDrive2.setPower(power);
        rightDrive1.setPower(power);
        rightDrive2.setPower(power);
    }

    // THIS IS WHERE WE WILL ADD THE AUTONOMOUS STEPS
    static void addAutonomousSteps(List<AutonomousStep> steps) {
        addForwardSteps(steps, 1500);
    }

    static void addForwardSteps(List<AutonomousStep> steps, long ms) {
        steps.add(new StepSetDriveMotorPower(1.0));
        steps.add(new StepSleep(ms));
        steps.add(new StepSetDriveMotorPower(0.0));
    }

    interface AutonomousStep {
        void run(AutonomousOpMode mode);
    }

    static class StepSetDriveMotorPower implements  AutonomousStep {
        double drivePower;
        StepSetDriveMotorPower(double drivePower) {
            this.drivePower = drivePower;
        }
        @Override
        public void run(AutonomousOpMode mode) {
            mode.setAllDriveMotorPower(drivePower);
        }
    }

    static class StepSleep implements AutonomousStep {
        long ms;
        StepSleep(long ms) {
            this.ms = ms;
        }
        @Override
        public void run(AutonomousOpMode mode) {
            mode.sleep(ms);
        }
    }
}