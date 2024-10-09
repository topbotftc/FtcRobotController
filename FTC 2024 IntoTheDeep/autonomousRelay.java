package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class autonomousRelay {

    // Circumference = 1.890
    // Ticks per revolution = 2000
    // Distance travelled = t/2000.0 * 1.89

    public static void goForwardIn(double inchesForward) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < inchesForward) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = Math.pow((inchesForward - distanceTravelled(ticks)) / 16.0, (.2));
            Robot.drive(speed, speed, speed, speed);
        }

        Robot.drive(0, 0, 0, 0);
    }

    public static void goRightIn(double inchesToTheRight) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < inchesToTheRight) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = Math.pow((inchesToTheRight - distanceTravelled(ticks)) / 16.0, (.2));
            Robot.drive(speed, speed, speed, speed);
        }

        Robot.drive(0, 0, 0, 0);
    }

    public static void turnRightDeg(double turnToTheRight) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            ticks = Robot.backleft.getCurrentPosition();
            double speed = 1.2 * Math.pow((turnToTheRight - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(speed, -speed, -speed, speed);

        Robot.drive(0, 0, 0, 0);
    }


    public static void turnLeftDeg(double turnToTheLeft) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            ticks = Robot.backleft.getCurrentPosition();
            double speed = 1.2 * Math.pow((turnToTheLeft - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(-speed, speed, speed, -speed);


        Robot.drive(0, 0, 0, 0);
    }


    public static double distanceTravelled(double ticks) {

        return ticks / 2000.0 * 1.89;         // inches
    }

    public static void main(String[] args) {

        while(true) {
            autonomousRelay.goForwardIn(9);
            autonomousRelay.goRightIn(9);
            autonomousRelay.goForwardIn(-9);
            autonomousRelay.goRightIn(-9);
            autonomousRelay.turnRightDeg(90);
            autonomousRelay.turnLeftDeg(90);

        }

    }



}
