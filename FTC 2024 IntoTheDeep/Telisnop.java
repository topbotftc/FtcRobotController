package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Telesnop")
public class Telisnop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);

        boolean slow = true;

        //defining things
        while (opModeIsActive()){

            boolean up= gamepad1.dpad_up;
            boolean down= gamepad1.dpad_down;
            boolean left= gamepad1.dpad_left;
            boolean right= gamepad1.dpad_right;

            boolean options = gamepad1.options;
            boolean previousOptions = gamepad1.options;

            double LSX = gamepad1.left_stick_x;
            double LSY = gamepad1.left_stick_y;
            double RSX = gamepad1.right_stick_x;



            //gearshift for joystick is ds, gearshift for dpad drive is bs
            // this is so you can change the speeds for both Ex: the joystick speed at 0.75 and the dpad speed at 1
            // change gear with options button
            double ds = 1;
            double bs = 1;
            if ((options) && !previousOptions) {
                slow = !slow;
            }
            previousOptions = options;

            ds = slow ? .5 : 1;
            bs = slow ? .5 : 1;

            //joystick drive
            if (Math.abs(LSX) > 0.05 || Math.abs(LSY) > 0.05 || Math.abs(RSX) > 0.05) {


                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * ds;
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x / 2;

                double v1 = r * Math.cos(robotAngle) + rightX; //lf // wsa cos
                double v2 = r * Math.sin(robotAngle) - rightX; //rf // was sin
                double v3 = r * Math.sin(robotAngle) + rightX; //lb // was sin
                double v4 = r * Math.cos(robotAngle) - rightX; //rb // was cos

                drive( v1,  v2,  v3,  v4);

            }
            //dpad drive
            else if(up) {
                drive(bs, bs, bs, bs);
            }
            else if(down) {
                drive(-bs, -bs, -bs, -bs);
            }
            else if(left) {
                drive(-bs, bs, -bs, bs);
            }
            else if(right) {
                drive(bs, -bs, bs, -bs);
            }
            else {
                int i = 0;
                drive(i, i, i, i);

            }







    }
}}
