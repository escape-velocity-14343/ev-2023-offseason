package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;

public class MotionProfile {
    double start, end, length;

    InterpLUT lut = new InterpLUT();
    public MotionProfile(double start, double end, double length) {
        setCoefficients(start, end, length);
    }
    public void setCoefficients(double start, double end, double length) {
        if (start>end) {
            throw new RuntimeException("Start must be before end");
        }
        if (length>end) {
            throw new RuntimeException("End is out of bounds");
        }
        this.start = start;
        this.end = end;
        this.length = length;
        lut.add(0,0);
        lut.add(start,1);
        lut.add(end,1);
        lut.add(length,0);

        lut.createLUT();
    }

    public double calculate(double distance, double distanceTraveled) {
        if (distanceTraveled<start)
            return lut.get(Math.abs(distanceTraveled))*Math.signum(distance);
        if (distanceTraveled>start && distance>end) {
            return (1*Math.signum(distance));
        }
        return lut.get(Math.abs(length-distance))*Math.signum(distance);
    }
    public double calculateAccel(double distance) {
        return lut.get((Math.abs(distance)-lut.get(Math.abs(distance)-0.1))*10)*Math.signum(distance);
    }

}
