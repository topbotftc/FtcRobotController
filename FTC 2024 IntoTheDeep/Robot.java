package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot{
    static DcMotor frontright;
    static DcMotor frontleft;
    static DcMotor backright;
    static DcMotor backleft;

    public static void initMotors(OpMode opMode){
        frontright = opMode.hardwareMap.get(DcMotor.class, "frontright");
        frontleft = opMode.hardwareMap.get(DcMotor.class, "frontleft");
        backright = opMode.hardwareMap.get(DcMotor.class, "backright");
        backleft = opMode.hardwareMap.get(DcMotor.class, "backleft");

        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public static void drive(double fl, double fr, double br, double bl){

        frontleft.setPower(fl);
        frontright.setPower(fr);
        backleft.setPower(br);
        backright.setPower(bl);


    }







    }
