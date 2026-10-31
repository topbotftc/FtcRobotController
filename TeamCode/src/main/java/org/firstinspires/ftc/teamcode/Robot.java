package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Robot {


    public static double slow = 0.67;
    public static double fast = 1;

    public static DcMotorEx rf;
    public static DcMotorEx rb;
    public static DcMotorEx lb;
    public static DcMotorEx lf;

    public static DcMotorControllerEx dcMotorControllerEx;



    public static Servo camServo;

    //public static HuskeyAiCamera huskCam;

    public static double gear = 1;
    public static boolean autoTurning = false;
    private static double gearprev = 1;


    public static GoBildaPinpointDriver odo;

    public static AutonomousDrive2 ad2;


    public static double[] targetPosRed = new double[]{60,-60,41};
    public static double[] targetPosBlue = new double[]{60,60,41};


    //Zones the color can't go
    public static double[][] noGoZoneRed = new double[][] {{5,-40}, {-72,-40}};
    public static double[][] noGoZoneBlue = new double[][] {{5,40}, {-72,40}};


    //Blue Side is 0 and Red is 1
    public static int side = 0;


    public static DcMotor[] motors = new DcMotor[4];



    public static void initAll(LinearOpMode opmode, int sideNum){
        initAll(opmode, sideNum, false);
    }


    //mode,false is Auto, true is telop
    public static void initAll(LinearOpMode opmode, int sideNum, boolean mode){
        initDrive(opmode);
        side = sideNum;

        ad2 = new AutonomousDrive2(opmode, mode);
        odo = ad2.getPinPoint();

    }




    public static void initDrive(LinearOpMode opmode) {
        rf = opmode.hardwareMap.get(DcMotorEx.class, "rf");
        rb = opmode.hardwareMap.get(DcMotorEx.class, "rb");
        lb = opmode.hardwareMap.get(DcMotorEx.class, "lb");
        lf = opmode.hardwareMap.get(DcMotorEx.class, "lf");



        motors[0] = rf;
        motors[1] = rb;
        motors[2] = lb;
        motors[3] = lf;



        //0 ports is the back motors config and both motors are going to both control and expansion hub
        //Encoders go to those ports


        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    //Getters and Setters and basic functions
    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {
        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);
    }

    public static void driveFC(LinearOpMode opMode){
        odo.update();

        double targetX = odo.getPosition().getX(DistanceUnit.INCH);
        double targetY = odo.getPosition().getY(DistanceUnit.INCH);


        double angle = 0;





        double currentHeadingRad = Math.toRadians(odo.getHeading(AngleUnit.DEGREES));

        if(side == 0){
            currentHeadingRad -= Math.PI/2;
        }else if(side == 1){
            currentHeadingRad += Math.PI/2;
        }


        double v1 = 0;// lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb


        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x;
        double rx = 0;



        if(opMode.gamepad1.right_bumper){
            rx = -ad2.turnPID(ad2.getAngleToGo(odo.getHeading(AngleUnit.DEGREES)+180,angle));
            opMode.telemetry.addData("angle to go: ", rx);
            opMode.telemetry.update();
        }else{
            rx = opMode.gamepad1.right_stick_x;
        }

       /* if(side == 1 && !opMode.gamepad2.dpad_up){
            odo.update();
            if((ad2.getX() < noGoZoneRed[0][0] && ad2.getY() < noGoZoneRed[0][1]) || (ad2.getX() < noGoZoneRed[1][0] && ad2.getY() < noGoZoneRed[1][1])) {
                y = -Math.abs(y);
            }
        }else if(side == 0 && !opMode.gamepad2.dpad_up){
            odo.update();
            if((ad2.getX() < noGoZoneBlue[0][0] && ad2.getY() > noGoZoneBlue[0][1]) || (ad2.getX() < noGoZoneBlue[1][0] && ad2.getY() > noGoZoneBlue[1][1])) {
                y = -Math.abs(y);
            }
        }
♦
        */



        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
        double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        v1 = (rotY + rotX + rx) / denominator;
        v3 = (rotY - rotX + rx) / denominator;
        v2 = (rotY - rotX - rx) / denominator;
        v4 = (rotY + rotX - rx) / denominator;


        if(opMode.gamepad1.right_bumper||Math.abs(opMode.gamepad1.right_stick_x) >= 0.05 || Math.abs(opMode.gamepad1.left_stick_x) >= 0.05 || Math.abs(opMode.gamepad1.left_stick_y) >=0.05){
            Robot.drive(v2, v4, v3, v1);
        } else if (opMode.gamepad1.dpad_up) {
            Robot.drive(0.3, 0.3, 0.3, 0.3);
        }else if (opMode.gamepad1.dpad_down) {
            Robot.drive(-0.3, -0.3, -0.3, -0.3);
        } else if (opMode.gamepad1.dpad_right) {
            Robot.drive(-0.3, 0.3, -0.3, 0.3);
        }else if (opMode.gamepad1.dpad_left) {
            Robot.drive(0.3, -0.3, 0.3, -0.3);
        }else {
            Robot.drive(0,0,0,0);
        }
    }
    private servo left;
    private servo right;





}