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
    public void podPid(double wheel, double heading) {
        double rotation = rot.getDegrees();
        double moveTo = AngleUnit.normalizeDegrees(heading-rotation);
        if (!compare(moveTo,0,90)) {
           // moveTo = AngleUnit.normalizeDegrees(moveTo-180);
            wheel*=-1;
        }
        telemetry.addData("moveTo", moveTo);
        telemetry.addData("rotation", rotation);
        telemetry.addData("heading", heading);
        pid.setSetPoint(moveTo);
        podMove(wheel, pid.calculate());
    }
    public void podMove(double wheel, double heading) {
        telemetry.addData("wheel", wheel);
        telemetry.addData("heading", heading);
        double topP = wheel-heading;
        double bottomP = wheel+heading;

        /*if (Math.abs(topP)>1) {
            bottomP = Math.abs(bottomP/topP)*Math.signum(bottomP);
            topP = Math.signum(topP);
        }
        if (Math.abs(bottomP)>1) {
            topP = Math.abs(topP/bottomP)*Math.signum(topP);
            bottomP = Math.abs(bottomP);
        }
        telemetry.addData("TopP", topP);
        telemetry.addData("bottomP", bottomP);
        telemetry.addData("Difference", Math.abs((topP-bottomP)/2));
        telemetry.addData("Heading", heading);
        // sanity check!
        topP = Math.min(1, topP);
        topP = Math.max(-1, topP);

        bottomP = Math.min(1, bottomP);
        bottomP = Math.max(-1, bottomP);*/
        double [] powers = {topP, bottomP};
        powers = normalize(powers, 1);
        telemetry.addData("TopP", topP);
        telemetry.addData("bottomP", bottomP);
        top.set(powers[0]);
        bottom.set(powers[1]);
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
}

