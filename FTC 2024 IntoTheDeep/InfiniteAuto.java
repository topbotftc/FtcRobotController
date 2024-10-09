package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Inf Auto")
public class InfiniteAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

// Point 0 to point 1 starting to score area, drop off specimien
        autonomousRelay.goRightIn(-36);
        autonomousRelay.goForwardIn(32);



// Point 1 to point 2
        autonomousRelay.goRightIn(0);
        autonomousRelay.goForwardIn(-33);

// Point 2 to point 3
        autonomousRelay.goRightIn(44);


    }

    public void goForwardIn(double inchesForward) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < inchesForward && opModeIsActive()) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = Math.pow((inchesForward - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(speed, speed, speed, speed);
        }

        Robot.drive(0, 0, 0, 0);
    }

    public void goRightIn(double inchesToTheRight) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < inchesToTheRight && opModeIsActive()) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = 1.2 * Math.pow((inchesToTheRight - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(speed, -speed, speed, -speed);
        }

        Robot.drive(0, 0, 0, 0);
    }

    public void turnRightDeg(double turnToTheRight) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < turnToTheRight && opModeIsActive()) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = 1.2 * Math.pow((turnToTheRight - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(speed, -speed, -speed, speed);
        }

        Robot.drive(0, 0, 0, 0);
    }


    public void turnLeftDeg(double turnToTheLeft) {
        double ticks = 0;
        Robot.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (distanceTravelled(ticks) < turnToTheLeft && opModeIsActive()) {
            ticks = Robot.backleft.getCurrentPosition();
            double speed = 1.2 * Math.pow((turnToTheLeft - distanceTravelled(ticks)) / 16.0, (0.27352297593));
            Robot.drive(-speed, speed, speed, -speed);
        }

        Robot.drive(0, 0, 0, 0);
    }

    //how to turn?

    public static double distanceTravelled(double ticks) {

        return ticks / 2000.0 * 1.89;         // inches
    }
}
