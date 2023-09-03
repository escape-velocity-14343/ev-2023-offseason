package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SwerveControl.compare;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class SwerveModule {
    ModuleState moduleState = new ModuleState();
    Motor top,bottom;
    AS5600 rot;
    Telemetry telemetry;
    PIDController pid = new PIDController(0.008, 0.1, 0);
    double error = 0;
    public SwerveModule(Motor top, Motor bottom, AS5600 rot, Telemetry telemetry) {
        this.top = top;
        this.bottom = bottom;
        this.rot = rot;
        this.telemetry = telemetry;
    }
    public void setPid(double p, double i, double d) {
        this.pid.setPID(p, i, d);
    }
    public void podMove(ModuleState module) {

    }
    public void podPid(Vector2d vector) {
        podPid(vector.magnitude(), vector.angle());
    }
    public void podPidXY(double x, double y) {
        podPid(Math.hypot(x,y), Math.toDegrees(Math.atan2(y,x)));
    }
    public void podPid(double wheel, double heading) {
        double rotation = rot.getDegrees();
        double moveTo = AngleUnit.normalizeDegrees(heading-rotation);
        if (!compare(moveTo,0,90)) {
            moveTo = AngleUnit.normalizeDegrees(moveTo-180);
            wheel*=-1;
        }
        telemetry.addData("moveTo", moveTo);
        telemetry.addData("rotation", rotation);
        telemetry.addData("heading", heading);
        pid.setSetPoint(moveTo);
        error = pid.getPositionError();

        podMove(Math.cos(Math.toRadians(error))*wheel, pid.calculate());
    }
    public void podMove(double wheel, double heading) {
        heading*=-1;
        telemetry.addData("wheel", wheel);
        telemetry.addData("heading", heading);
        double topP = wheel-heading;
        double bottomP = wheel+heading;


        double [] powers = {topP, bottomP};
        powers = normalize(powers, 1);
        telemetry.addData("TopP", topP);
        telemetry.addData("bottomP", bottomP);
        top.set(powers[0]);
        bottom.set(-powers[1]);
    }

    public double[] normalize(double[] values, double magnitude) {
        double maxMagnitude = Math.abs(values[0]);
        for (int i = 1; i < values.length; i++) {
            double temp = Math.abs(values[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude>magnitude) {
            for (int i = 0; i < values.length; i++) {
                values[i] = (values[i] / maxMagnitude) * magnitude;
            }
        }
        return values;

    }
    public boolean close(double moveTo) {
        return compare(moveTo, rot.getDegrees(),10)||compare(moveTo, AngleUnit.normalizeDegrees(rot.getDegrees()+180), 10);
    }
    public double getError() {
        return error;
    }

}

