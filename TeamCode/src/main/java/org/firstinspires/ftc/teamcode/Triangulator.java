package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VISIONTEST.xM;
import static org.firstinspires.ftc.teamcode.VISIONTEST.yM;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class Triangulator {
    double x=0;
    double y=0;
    double z=0;
    //adding offsets
    double robotX=0;
    double robotY=0;
    double mountingHeight=0;
    double robotHeading=0;
    double cameraAngle=0;

    public Triangulator () {

    }

    public void triangulateFromPixels(int cx, int cy, double width) {
        double multiplier = 1/640.0;

        double distance = (-12*width/112.0)+36;
        double distancePlane = (Math.sqrt(1+multiplier*multiplier*(cx*cx+cy*cy)))*distance;

         x = cx*distancePlane*xM;
         y = cy*distancePlane*yM;
         z = distancePlane;

         z = z * Math.cos(cameraAngle) - y * Math.sin(cameraAngle);
         y = z * Math.sin(cameraAngle) + y * Math.cos(cameraAngle);
         x = x * Math.cos(robotHeading) - z * Math.sin(robotHeading);
         z = x * Math.sin(robotHeading) + z * Math.cos(robotHeading);

         double oldZ = z;
         z = y;
         y = x;
         x = oldZ;

         x+=robotX;
         y+=robotY;
         z+=mountingHeight;



    }
    public void setRobotPose(Pose2d robotPose) {
        robotX = robotPose.getX();
        robotY = robotPose.getY();
        robotHeading = robotPose.getRotation().getRadians();
    }
    public void setRobotPose(double x, double y, double heading) {
        robotX = x;
        robotY= y;
        robotHeading = heading;
    }
    public void setCameraAngle(double angle) {
        cameraAngle = angle;
    }
    public void setCameraHeight(double height) {
        mountingHeight = height;
    }
    public void triangulateStereo(int x1, int y1, int x2, int y2) {
        //TODO
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
}
