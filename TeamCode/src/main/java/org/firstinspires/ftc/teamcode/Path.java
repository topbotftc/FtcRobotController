package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutonomousDrive2;

import java.util.ArrayList;

public class Path{


    AutonomousDrive2 ad;

    ArrayList<double[]> path = new ArrayList<>();
    LinearOpMode opMode;

    private DistanceUnit unit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    double[] startPose;



    public Path(AutonomousDrive2 ad, LinearOpMode opMode, double[] startPose, double startTangent){
        this.ad = ad;
        this.startPose = startPose;

        this.path.add(startPose);

        double[] controlPoint1 = makeControlPoint(startPose, startTangent + 180, 1);

        this.path.add(controlPoint1);
        this.opMode = opMode;
    }

    public void setUnit(DistanceUnit unit){
        this.unit = unit;
    }

    public int getLength(){
        return path.size();
    }


    public void addSpline( double x, double y,double heading, double endTangent){
        double[] point2 = new double[]{x,y,heading};

        double[] controlPoint2 = makeControlPoint(point2, endTangent,1);
        double[] controlPoint3 = makeControlPoint(point2, endTangent + 180,1);

        path.add(controlPoint2);
        path.add(point2);
        path.add(controlPoint3);

    }

    public void addSpline( double x, double y,double heading, double endTangent, double weight){
        double[] point2 = new double[]{x,y,heading};

        double[] controlPoint2 = makeControlPoint(point2, endTangent, weight);
        double[] controlPoint3 = makeControlPoint(point2, endTangent + 180, weight);

        path.add(controlPoint3);
        path.add(point2);
        path.add(controlPoint2);

    }



    public boolean isComplete(double tick,Path path, double[] point, AutonomousDrive2 ad2){

        double dist1 = Math.abs(point[0] -ad2.getX());
        double dist2 = Math.abs(point[1] -ad2.getY());



        return (tick >= (path.getLength()-1)/4 && dist1 <= ad2.POS_ERROR_TOLERANCE2 && dist2 <= ad2.POS_ERROR_TOLERANCE2);

    }
    private double[] pathFunc(double t, int index){
        index *= 3;
        index = Math.min(index, path.size() - 5);
        double[] point = splineFunc(path.get(index), path.get(index+1), path.get(index+2), path.get(index+3),t);
        return point;
    }

    public static double[] splineFunc(double[] point1, double[] point2,double[] point3,double[] point4,double t) {


        double angle1 = point1[2];
        double angle2 = point2[2];

        double[] p1 =  blend(point1, point2, t);
        double[] p2 =  blend(point2, point3, t);
        double[] p3 =  blend(point3, point4, t);
        double[] p4 =  blend(p1, p2, t);
        double[] p5 =  blend(p2, p3, t);
        double[] p6 =  blend(p4, p5, t);

        double[] point = p6;
        return point;


    }
    private static double[] blend(double[] point1, double[] point2,double t){
        double x = (1-t)*point1[0]+t*point2[0];
        double y = (1-t)*point1[1]+t*point2[1];
        double heading = (1-t)*point1[2]+ t*point2[2];

        double[] point = new double[]{x,y,heading};
        return point;
    }

    private static double[] makeControlPoint(double[] point1,double angle1, double weight){
        double x = point1[0] + weight * Math.sin(Math.toRadians(angle1));
        double y = point1[1] -  weight * Math.cos(Math.toRadians(angle1));;

        double[] point = new double[]{x,y,point1[2]};
        return  point;

    }
    public double[] getPoint(double tick){
        int index = (int)(Math.floor(tick));
        double t = tick - index;
        return pathFunc(t,index);
    }





}