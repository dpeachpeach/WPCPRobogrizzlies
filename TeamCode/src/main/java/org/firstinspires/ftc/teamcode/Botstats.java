package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Botstats {

    DcMotor leftDrive1;
    DcMotor rightDrive1;
    DcMotor leftDrive2;
    DcMotor rightDrive2;

    Botstats(HardwareMap hardwareMap){
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_back_drive");

        //Direction
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);

    }



}
