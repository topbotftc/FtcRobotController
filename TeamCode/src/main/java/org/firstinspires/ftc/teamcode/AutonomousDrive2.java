
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;


/*

Parker Pruitt, 2025
FTC team Ubett 8672

A autonomous library for movement based on position
using the Goblida Pinpoint Co-Processor

The coordinate System is (0,0) is the left corner of your starting side
Each tile is 24 inches
X is vertical forward backward axis
Y is the horizontal side to side axis
Standard (x,y) -> Our coordinates are (y,x)

 */
public class AutonomousDrive2 {



    //Error Tolerances
    public final double POS_ERROR_TOLERANCE = 0.25;
    public final double HEADING_ERROR_TOLERANCE = 0.75;

    public final double POS_ERROR_TOLERANCE2 = 2.5;

    private final double MAX_MOTOR_CURRENT = 9.5;
    private final double DEAD_WHEEL_RADIUS_MM = 16;
    //PID controls

    //For Drive Movement
    private double kDP = 0.025;//0.025
    private double kDI = 0.005;//0.005
    private double kDD = 0.0025;//0.0025
    public double errorSumDX = 0;
    public double errorSumDY = 0;
    public double errorSumRangeD = 4;

    //splines
    private double kDP2 = 0.14;//0.14
    private double kDI2 = 0.005;//0.005
    private double kDD2 = 0.0015;//0.0025
    public double errorSumDX2 = 0;
    public double errorSumDY2 = 0;
    public double errorSumRangeD2 = 4;

    //For Turn Movement
    public double kTP = 0.016;//0.016
    public double kTI = 0.005;//0.002
    public double kTD = 0.0015;//0.003
    public double errorSumT = 0;
    public double errorSumRangeT = 5;



    //Drive motor names
    private String leftFrontName = "lf";
    private String leftBackName = "lb";
    private String rightFrontName = "rf";
    private String rightBackName = "rb";

    //Drive motor vars
    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;

    public int leftFrontNum;
    public int leftBackNum;
    public int rightFrontNum;
    public int rightBackNum;

    public static DcMotorControllerEx motorControllerEx;

    private static double[] motorCurrents = new double[4];

    //Gobilda Pinpoint
    public String odoName = "odo";
    public static GoBildaPinpointDriver odo;

    //Pinpoint offsets from Center in mm and encoder Direction
    //Left is +X, Front is +Y
    public double xOffset = 152.4;//-203mm
    public double yOffset = -43.18;//-24.5mm

    public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    //Dead Wheel Type

    public GoBildaPinpointDriver.GoBildaOdometryPods encoderPod = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    //LinearOpmode

    private LinearOpMode opMode;


    //Start Positions as {y,x} format and add to the master list for poses

    private double[] pos1 = new double[] {0,0,0};
    private double[] pos2 = new double[] {36,8, 0 };
    private double[] pos3 = new double[] {72,8, 180 };


    private double[][] masterStartPoses = new double[][] { pos1, pos2,pos3};


    //Background Varables

    public static double timeLimit = 0;
    public static double timeLimit2 = 0;
    public static boolean outInfo = true;
    public static double currentLimit = 1;

    private static int startPose = 1;

    //Control Hub and Expasion Hub
    private LynxModule controlHub, expansionHub;

    public ArrayList<Integer> motorNums = new ArrayList<>();

    boolean atTarget = false;

    double targetTime = 0;

    public FtcDashboard dashboard;

    public Telemetry telemetryd;

    ArrayList<Path> paths = new ArrayList<>();




    public AutonomousDrive2(LinearOpMode opMode, boolean isTelop) {
        this.opMode = opMode;

        controlHub = (LynxModule) (opMode.hardwareMap.get(LynxModule.class, "Control Hub"));
        //expansionHub = (LynxModule)(opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 2"));

        dashboard = FtcDashboard.getInstance();
        telemetryd = dashboard.getTelemetry();

        lf = Robot.lf;
        lb = Robot.lb;
        rb = Robot.rb;
        rf = Robot.rf;


        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, odoName);
        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y

        if (!isTelop) {
            odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
            odo.setEncoderDirections(xDirection, yDirection);
            odo.setEncoderResolution(encoderPod);

            odo.recalibrateIMU();
            opMode.sleep(200);
            odo.resetPosAndIMU();
            opMode.sleep(200);
            odo.setHeading(0, AngleUnit.DEGREES);
        }






    }




    //Getters and Setters and basic functions
    public void drive(double rfPower, double rbPower, double lbPower, double lfPower) {

        Robot.drive(rfPower,rbPower,lbPower,lfPower);
    }

    public GoBildaPinpointDriver getPinPoint(){return odo; }

    public DcMotorEx getMotor(int num){
        switch (num){
            case 0:
                return lf;
            case 1:
                return lb;
            case 2:
                return rf;
            case 3:
                return rb;

        }
        return lf;
    }

    public void resetOdo(LinearOpMode opMode, double x, double y, double heading){ // left is 8.5x and 36y // right is __x and __y
        odo.recalibrateIMU();
        opMode.sleep(500);
        odo.resetPosAndIMU();
        opMode.sleep(400);
        odo.setPosition(new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES, heading));
        opMode.sleep(400);
    }


    public Pose2D getPos(){odo.update(); return  odo.getPosition(); }
    public double getY(){odo.update(); return  odo.getPosition().getY(DistanceUnit.INCH); }
    public double getX(){odo.update(); return odo.getPosition().getX(DistanceUnit.INCH); }

    //getHeading outputs 0-360
    public double getHeading(){
        double rawHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        return  rawHeading + 180;
    }
    //getHeadingNorm outputs -180-180
    public double getHeadingNorm(){
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }
    //getHeadingUnNorm outputs -inf-inf
    public double getHeadingUnNorm(){
        return odo.getHeading(AngleUnit.DEGREES);
    }

    //Returns False if robot is not moving substantially
    public boolean isMoving(){
        return Math.hypot(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH)) > 0.005;
    }
    public boolean isStuck(double distanceToGo){
        return  !isMoving() && Math.abs(distanceToGo) < 1.75;
    }





    public double getAngleToGo(double currentHeading,double degrees){
        double angleTogo = degrees - currentHeading;

        if(Math.abs(angleTogo) > 180){
            if(currentHeading < 180){
                angleTogo = -((currentHeading) + (360 - degrees));
            }else{
                angleTogo = (degrees + (360 - currentHeading));
            }
        }
        return angleTogo;
    }

    public double[] getMotorCurrents(){
        double currentLF = motorControllerEx.getMotorCurrent(leftFrontNum, CurrentUnit.AMPS);
        double currentLB = motorControllerEx.getMotorCurrent(leftBackNum, CurrentUnit.AMPS);
        double currentRF = motorControllerEx.getMotorCurrent(rightFrontNum, CurrentUnit.AMPS);
        double currentRB = motorControllerEx.getMotorCurrent(rightBackNum, CurrentUnit.AMPS);
        return new double[] {currentRF, currentRB, currentLB, currentLF};
    }

    public ArrayList<Boolean> isMotorsOver(){
        boolean overCurrentLF = motorControllerEx.isMotorOverCurrent(leftFrontNum);
        boolean overCurrentLB = motorControllerEx.isMotorOverCurrent(leftBackNum);
        boolean overCurrentRF = motorControllerEx.isMotorOverCurrent(rightFrontNum);
        boolean overCurrentRB = motorControllerEx.isMotorOverCurrent(rightBackNum);

        ArrayList<Boolean> list = new ArrayList<>();

        list.add(overCurrentLF);
        list.add(overCurrentLB);
        list.add(overCurrentRF);
        list.add(overCurrentRB);

        return list;
    }

    public double getVeloTotal(){
        return Math.abs(Math.hypot(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH)));
    }


    public double[] getPID(int PIDNum) {
        switch (PIDNum) {
            case 0:
                return new double[]{kDP, kDI, kDD};
            case 1:
                return new double[]{kTP, kTI, kTD};

        }
        return new double[]{kDP, kDI, kDD};
    }

    public void setPID(double kP, double kI, double kD, int PIDNum){
        switch (PIDNum){
            case 0:
                kDP = kP;
                kDI = kI;
                kDD = kD;
                break;
            case 1:
                kTP = kP;
                kTI = kI;
                kTD = kD;
                break;

        }
    }


    //Set Limit to 0 if you don't want a time limit
    //Default is 0
    public void setTimeLimit(double time){
        timeLimit = time;
    }
    public void setTimeLimit2(double time){
        timeLimit2 = time;
    }

    public boolean checkTime(double startTime, double currentTime){
        if(Math.abs(currentTime - startTime) >= timeLimit){
            return true;
        }else {
            return false;
        }
    }

    public void setOutputInfo(boolean val){
        outInfo = val;
    }

    public void outputInfo(){
        if(outInfo){
            opMode.telemetry.addData("X pos: ", getX());
            opMode.telemetry.addData("Y pos: ", getY());
            opMode.telemetry.addData("Heading: ", getHeading());
            opMode.telemetry.addData("Heading Norm: ", getHeadingNorm());
            opMode.telemetry.update();

        }
    }

    public void outputInfo(String[] names, double[] vals){
        for(int i = 0; i < names.length; i++){
            telemetryd.addData(names[i], vals[i]);
        }
        telemetryd.update();
    }
    public void outputInfo(double xdist, double ydist){
//        opMode.telemetry.addData("distanceToGo", distanceToGo);
//        opMode.telemetry.addData("distanceTotal", distanceTotal);
//        opMode.telemetry.addData("power", power);
////        opMode.telemetry.addData("get x", this.getX());
////        opMode.telemetry.addData("get y", this.getY());
//        opMode.telemetry.addData("heading ", odo.getPosition().getHeading(AngleUnit.DEGREES));
//        opMode.telemetry.addData("is moving", isMoving());
//        opMode.telemetry.update();


        telemetryd.addData("X distance: ", xdist);
        telemetryd.addData("Y distance: ", ydist);
        telemetryd.update();


    }

    public void outPutDriveInfo(ArrayList<String> names,ArrayList<Double> list){
        opMode.telemetry.clearAll();
        for(int i = 0; i < Math.min(list.size(),names.size()); i++){
            opMode.telemetry.addData(names.get(i), list.get(i));
        }
        opMode.telemetry.update();
    }

    public double turnSlope(double targetHeading, double startHeading, double currentDist, double startDist){
        double slope = ((startHeading-targetHeading)/(startDist));

        return slope * (currentDist - startDist) + startHeading;
    }

    public int inchesToTicks(double dist){
        return (int)((dist * 25.4 * 2000)/ (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI));
    }

    public double ticksToInches(int ticks){
        return (ticks * (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI))/(25.4 * 2000);
    }


    //PIDs


    public static double powerCurvingOrg(double distanceToGo){
        double slope = 18;
        double max = .80;
        double min = 0;
        if(distanceToGo > 0) {
            return Math.max(Math.min(distanceToGo / slope, max), min);
        } else {
            return Math.min(Math.max(distanceToGo / slope, -max), -min);
        }
    }

    public static double powerCurvingTurnOrg(double angleToGo){
        double slope = 90;
        double min = 0.05;
        if(angleToGo > 0) {
            return (angleToGo / slope < min) ? min : angleToGo / slope;
        }else{
            return (angleToGo / slope > -min) ? -min : angleToGo / slope;
        }
    }

    public double movePID(double error, String axis){
        odo.update();


        double output = error * kDP;
        if(axis.toLowerCase().charAt(0) == 'y') {
            output -= odo.getVelY(DistanceUnit.INCH) * kDD;
            if (Math.abs(error) <= errorSumRangeD) {
                output += errorSumDY * kDI;
                errorSumDY += error;

            }else{
                errorSumDY = 0;
            }
        }
        if(axis.toLowerCase().charAt(0) == 'x'){
            output -= odo.getVelX(DistanceUnit.INCH) * kDD;
            if(Math.abs(error) <= errorSumRangeD){
                output += errorSumDX * kDI;
                errorSumDX+= error;
            }else{
                errorSumDX =0;
            }
        }


        return output;
    }

    public double movePIDSpline(double error, String axis){
        odo.update();


        double output = error * kDP2;

        if(error > 5){
            return movePID(error, axis);
        }
        if(axis.toLowerCase().charAt(0) == 'y') {
            output -= odo.getVelY(DistanceUnit.INCH) * kDD2;
            if (Math.abs(error) <= errorSumRangeD2) {
                output += errorSumDY * kDI2;
                errorSumDY2 += error;

            }else{
                errorSumDY2 = 0;
            }
        }
        if(axis.toLowerCase().charAt(0) == 'x'){
            output -= odo.getVelX(DistanceUnit.INCH) * kDD2;
            if(Math.abs(error) <= errorSumRangeD2){
                output += errorSumDX * kDI2;
                errorSumDX2+= error;
            }else{
                errorSumDX2 =0;
            }
        }


        return output;

    }

    public double turnPID(double error){
        odo.update();
        double output = error * kTP - odo.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized()) * kTD + errorSumT*kTI;
        if(Math.abs(error) <= errorSumRangeT){
            errorSumT += error;
        }else{
            errorSumT = 0;
        }
        return output;
    }


    //Movement

    public void forward(double distanceTotal) {
        double startTime = opMode.time;
        double startpos = odo.getPosition().getX(DistanceUnit.INCH);
        double distanceToGo = (distanceTotal);
        while (Math.abs(distanceToGo) > (POS_ERROR_TOLERANCE + 0.13) && opMode.opModeIsActive()) {
            distanceToGo = distanceTotal - (odo.getPosition().getX(DistanceUnit.INCH) - startpos);
            double power = movePID(distanceToGo, "y");
            drive(power, power, power, power);

        }
        drive(0,0,0,0);

    }
    public void goToPointConstantHeading(double targetX, double targetY){
        odo.update();

        errorSumDX = 0;
        errorSumDY = 0;
        errorSumT = 0;


        double targetXDist =  (targetX- getX());
        double targetYDist = (getY() - targetY);
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = turnPID(startHeading);

        double startTime = opMode.time;
        double startTime2 = opMode.time;

        targetTime = opMode.time;


        while(!checkTime(startTime,opMode.time) &&
                (Math.abs(targetXDist) > POS_ERROR_TOLERANCE ||
                        Math.abs(targetYDist) > POS_ERROR_TOLERANCE)

                && opMode.opModeIsActive()) {





            odo.update();




            targetXDist =(targetX - getX());
            targetYDist = -(targetY - getY());
            totalDist = Math.hypot(targetXDist, targetYDist);

            outputInfo(targetXDist,targetYDist);

            opMode.telemetry.clearAll();
            opMode.telemetry.addData("Out X: " ,targetXDist );
            opMode.telemetry.addData("Out Y: " ,targetYDist );
            opMode.telemetry.update();

            double currentHeadingRad = Math.toRadians(getHeadingNorm());

            double angleToGo = getAngleToGo(getHeading(), startHeading);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = movePID(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist,"x");
            double rx = -turnPID(getAngleToGo(getHeading(), startHeading));

            if(rx < 0){
                rx = Math.max(-0.05, rx);
            }else{
                rx = Math.min(0.05, rx);
            }

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



            drive(v2,v4,v3,v1);
        }
        drive(0,0,0,0);
    }

    public void goToPointConstantHeadingOrg(double targetX, double targetY){

        odo.update();

        errorSumDX = 0;
        errorSumDY = 0;
        errorSumT = 0;

        double targetXDist = (targetX - getX());
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);



        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();

            targetXDist = (targetX - getX());
            targetYDist = (targetY - getY());
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = -Math.toRadians(getHeadingNorm());


            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = powerCurvingOrg(targetXDist); // Remember, Y stick value is reversed
            double x = powerCurvingOrg(targetYDist);
            double rx = 0;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(currentHeadingRad) - y * Math.sin(currentHeadingRad);
            double rotY = x * Math.sin(currentHeadingRad) + y * Math.cos(currentHeadingRad);


            rotX = rotX * 1.05;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(powerCurvingOrg(totalDist)) * (Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx)), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 =  (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
        drive(0,0,0,0);
    }

    public void goToPointLinear(double targetX, double targetY, double targetHeading){
        odo.update();

        errorSumDX = 0;
        errorSumDY = 0;
        errorSumT = 0;


        double targetXDist =  (targetX- getX());
        double targetYDist = (getY() - targetY);
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = -turnPID(startHeading);

        double startTime = opMode.time;
        double startTime2 = opMode.time;

        double startDist = totalDist;
        targetTime = opMode.time;


        while(!checkTime(startTime,opMode.time) &&
                (Math.abs(targetXDist) > POS_ERROR_TOLERANCE ||
                        Math.abs(targetYDist) > POS_ERROR_TOLERANCE)

                && opMode.opModeIsActive()) {



            odo.update();




            targetXDist =(targetX - getX());
            targetYDist = -(targetY - getY());
            totalDist = Math.hypot(targetXDist, targetYDist);


            opMode.telemetry.addData("Out X: " ,targetXDist );
            opMode.telemetry.addData("Out Y: " ,targetYDist );
            opMode.telemetry.update();

            double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);

            double angleToGo = getAngleToGo(getHeading(), currentTargetHead);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());


            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = movePID(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist,"x");
            double rx = -turnPID(angleToGo);


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



            drive(v2,v4,v3,v1);
        }
        drive(0,0,0,0);
    }


    public void goToPointLinearNoStop(double targetX, double targetY, double targetHeading){
        odo.update();

        errorSumDX2 = 0;
        errorSumDY2 = 0;
        errorSumT = 0;


        double targetXDist =  (targetX- getX());
        double targetYDist = (getY() - targetY);
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = -turnPID(startHeading);

        double startTime = opMode.time;
        double startTime2 = opMode.time;

        double startDist = totalDist;
        targetTime = opMode.time;


        while(!checkTime(startTime,opMode.time) &&
                (Math.abs(targetXDist) > POS_ERROR_TOLERANCE2 ||
                        Math.abs(targetYDist) > POS_ERROR_TOLERANCE2)

                && opMode.opModeIsActive()) {



            odo.update();




            targetXDist =(targetX - getX());
            targetYDist = -(targetY - getY());
            totalDist = Math.hypot(targetXDist, targetYDist);


            opMode.telemetry.addData("Out X: " ,targetXDist );
            opMode.telemetry.addData("Out Y: " ,targetYDist );
            opMode.telemetry.update();
            telemetryd.addData("Out X: " ,targetXDist );
            telemetryd.addData("Out Y: " ,targetYDist );
            telemetryd.addData("PX: ", targetXDist * kDP2);
            telemetryd.addData("DP: ", targetXDist * kDD2);
            telemetryd.addData("PY: ", targetYDist * kDP2);
            telemetryd.addData("DY: ", targetYDist * kDD2);
            telemetryd.update();

            double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);

            double angleToGo = getAngleToGo(getHeading(), currentTargetHead);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());


            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = movePIDSpline(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePIDSpline(targetYDist,"x");
            double rx = -turnPID(angleToGo);


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



            drive(v2,v4,v3,v1);
        }
    }



    public void goToPointLinearOrg(double targetX, double targetY, double targetHeading){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);

        double startDist = totalDist;

        double startHeading = getHeading();
        double headingdist = targetHeading - startHeading;
        double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);


        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(getHeading(), currentTargetHead)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();
            ;

            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());
            currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);
            double angleToGo = getAngleToGo(getHeading(), currentTargetHead);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = -powerCurvingOrg(targetXDist); // Remember, Y stick value is reversed
            double x = powerCurvingOrg(targetYDist);
            double rx = powerCurvingTurnOrg(angleToGo);
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



            drive(v2,v4,v3,v1);
        }
        opMode.sleep(50);
    }

    public void goToHeading(double heading){
        heading = Math.abs(heading) % 360;
        odo.update();
        double angleToGo = getAngleToGo(getHeading(), heading);
        double power;
        double startTime = opMode.time;
        errorSumT = 0;
        while(!checkTime(startTime, opMode.time) && Math.abs(angleToGo) > HEADING_ERROR_TOLERANCE && opMode.opModeIsActive()){
            odo.update();
            outputInfo();
            angleToGo = getAngleToGo(getHeading(), heading);
            outputInfo(getHeading(), heading);
            power = turnPID(angleToGo);
            drive(power, power,-power,-power);
        }
        drive(0,0,0,0);
    }

    public void goToHeadingOrg(double heading){
        heading = Math.abs(heading) % 360;
        odo.update();
        double angleToGo = getAngleToGo(getHeading(), heading);
        double power;
        double startTime = opMode.time;
        while(!checkTime(startTime, opMode.time) && Math.abs(angleToGo) > HEADING_ERROR_TOLERANCE && opMode.opModeIsActive()){
            odo.update();
            outputInfo();
            angleToGo = getAngleToGo(getHeading(), heading);
            power = powerCurvingTurnOrg(angleToGo);
            drive(power, power,-power,-power);
        }
        drive(0,0,0,0);
        opMode.sleep(50);
    }

    public boolean isAttarget(double targetX, double x, double targetY,double y, double time){
        if(Math.abs(targetX - x) <= POS_ERROR_TOLERANCE && Math.abs(targetY - y) <= POS_ERROR_TOLERANCE){
            return (Math.abs(time - targetTime) > 0.1);
        }else{
            targetTime = time;
        }

        return false;
    }

    public boolean timeCheck2(double startTime, double currentTime) {
        if (timeLimit2 == 0) {
            return false;
        }else {
            if ((currentTime - startTime) >= timeLimit2) {
                return true;
            } else {
                return false;

            }
        }

    }

    public void createPath(double[] startPose, double startTangent){
        Path path = new Path(this, opMode, startPose,startTangent);
        paths.add(path);
    }

    public Path getPath(int index){
        if(index < 0 || index >= paths.size()){
            return null;
        }
        return paths.get(index);
    }

    public void runPath(int pathNum){
        Path path = getPath(pathNum);
        double t = 0;
        while(!path.isComplete(t,path, path.path.get(path.getLength()-2), this) && opMode.opModeIsActive()){
            double[] point = path.getPoint(t);
            goToPointLinearNoStop(point[0],point[1], point[2]);
            t += 0.025;
        }
        double[] point =  path.path.get(path.getLength()-2);
        goToPointLinear(point[0], point[1], point[2]);
        opMode.sleep(50);

    }

    public void runPath(int pathNum, double pace){
        Path path = getPath(pathNum);
        double t = 0;
        pace = Math.abs(pace);
        while(!path.isComplete(t,path, path.path.get(path.getLength()-2), this) && opMode.opModeIsActive()){
            double[] point = path.getPoint(t);
            goToPointLinearNoStop(point[0],point[1], point[2]);
            t += pace;

        }
        double[] point =  path.path.get(path.getLength()-2);
        goToPointLinear(point[0], point[1], point[2]);
        opMode.sleep(50);

    }

    public void runPathConstantHeading(int pathNum, double pace){
        Path path = getPath(pathNum);
        odo.update();
        double startHeading = getHeading();
        double t = 0;
        pace = Math.abs(pace);
        while(!path.isComplete(t,path, path.path.get(path.getLength()-2), this) && opMode.opModeIsActive()){
            double[] point = path.getPoint(t);
            goToPointLinearNoStop(point[0],point[1], startHeading);
            t += pace;
            opMode.telemetry.addData("t: ", t);
        }
        double[] point =  path.path.get(path.getLength()-2);
        goToPointLinear(point[0], point[1], startHeading);
        opMode.sleep(50);

    }

}