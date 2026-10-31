package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Robot")
public class Testopmode extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this, 0);
        waitForStart();
        while(opModeIsActive()){
            double leftSide = gamepad1.left_stick_y;
            double rightSide = gamepad1.right_stick_y;

            drive(rightSide,rightSide, leftSide,leftSide);
            double xPos = ad2.getX();
            double yPos = ad2.getY();

            telemetry.addData("XPos", xPos);
            telemetry.addData("yPos", yPos);
            telemetry.update();
            ad2.setOutputInfo(true);

        }

    }
}
