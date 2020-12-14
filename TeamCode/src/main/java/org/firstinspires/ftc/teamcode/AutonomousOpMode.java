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

import android.text.method.MovementMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "AutonomousOpMode", group = "Autonomous")
public class AutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // Motor on the front left
    public DcMotor leftDrive1;
    // Motor on the front right
    public DcMotor rightDrive1;
    // Motor on the back left
    public DcMotor leftDrive2;
    // Motor on the back right
    public DcMotor rightDrive2;
    // Conveyor belt
    public DcMotor conveyorBelt;
    // Left shooter motor (ultraplanetary)
    public DcMotor leftShooter;
    // Right shooter motor (ultraplanetary)
    public DcMotor rightShooter;
    // Servo to turn the conveyor
    public Servo conveyorServo;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // DAVID PETRE: Initialize hardware vars, see note below.
        // Be advised, until robot is properly configured, the devicenames are PLACEHOLDERS.
        // We need to replace them with what we eventually write down when we config

        leftDrive1 = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_back_drive");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor");
        leftShooter = hardwareMap.get(DcMotor.class, "left_shooter");
        rightShooter = hardwareMap.get(DcMotor.class, "right_shooter");
        conveyorServo = hardwareMap.get(Servo.class, "floorAntiscuffer");
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);

        // runs the moment robot is initialized
        waitForStart();
        runtime.reset();
        conveyorServo.setPosition(0);


        // runs after driver presses play
        while (opModeIsActive()) {
            conveyorServo.setPosition(1);
            raiseConveyor();
            sleep(5000);
            Moving(24,-1);
            turn45(1);
            sleep(5000);
            Moving(12,-1);
            sleep(5000);
            raiseConveyor();
            sleep(5000);
            startConveyor();
            sleep(5000);
            startShooter();
            sleep(5000);
            stopShooter();
            sleep(5000);
            lateralDrive(6,1);

            jobsDone();
        }
    }


    public void Moving(int inches, int direction){
        if (direction == 1){
            leftDrive1.setPower(1);
            rightDrive1.setPower(1);
            leftDrive2.setPower(1);
            rightDrive2.setPower(1);
            sleep(inches * 100);
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
        }
        if (direction == -1){
            leftDrive1.setPower(-1);
            rightDrive1.setPower(-1);
            leftDrive2.setPower(-1);
            rightDrive2.setPower(-1);
            sleep(inches * 100);
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
        }
    }
    public void turn45(int direction){
        if (direction == 1){
            leftDrive1.setPower(-.5);
            leftDrive2.setPower(-.5);
            rightDrive1.setPower(.5);
            rightDrive2.setPower(.5);
            sleep(125);
        }
        if (direction == -1){
            leftDrive1.setPower(.5);
            leftDrive2.setPower(.5);
            rightDrive1.setPower(-.5);
            rightDrive2.setPower(-.5);
            sleep(125);
        }
    }
    public void jobsDone(){
        leftDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive1.setPower(0);
        rightDrive2.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        conveyorServo.setPosition(1);
        conveyorBelt.setPower(0);
    }

    public void lowerConveyor(){
        conveyorServo.setPosition(0);
    }
    public void raiseConveyor(){
        conveyorServo.setPosition(1);
    }
    public void startShooter(){
        leftShooter.setPower(-1);
        rightShooter.setPower(1);
    }
    public void stopShooter(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public void startConveyor(){
        conveyorBelt.setPower(1);
    }
    public void stopConveyor(){
        conveyorBelt.setPower(0);
    }

    public void lateralDrive(int inches, int direction){
        if (direction == 1){
        leftDrive1.setPower(-1);
        leftDrive2.setPower(1);
        rightDrive1.setPower(1);
        rightDrive2.setPower(-1);
        sleep(inches * 10);
        }
        if (direction == -1){
        leftDrive1.setPower(1);
        leftDrive2.setPower(-1);
        rightDrive1.setPower(-1);
        rightDrive2.setPower(1);
        sleep(inches * 10);
        }
    }
}