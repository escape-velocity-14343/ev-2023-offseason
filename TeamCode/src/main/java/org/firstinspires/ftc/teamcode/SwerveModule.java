package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class SwerveModule {
    ModuleState moduleState = new ModuleState();
    Motor top,bottom;
    AS5600 rot;
    PIDController pid = new PIDController(0.008, 0.1, 0);
    public SwerveModule(Motor top, Motor bottom, AS5600 rot) {
        this.top = top;
        this.bottom = bottom;
        this.rot = rot;
    }
    public void setPid(double p, double i, double d) {
        this.pid.setPID(p, i, d);
    }
    public void podMove(ModuleState module) {

    }
    public void podMove(double wheel, double heading) {
        double topP = wheel-heading;
        double bottomP = wheel+heading;
        if (topP>1) {
            bottomP = bottomP/topP;
            topP = topP/topP;
        }
        if (bottomP>1) {
            topP = topP/bottomP;
            bottomP = bottomP/bottomP;
        }
        top.set(topP);
        bottom.set(bottomP);
    }
}
class ModuleState {
    double targetRotation = 0;

    double wheelPower = 0;
    Translation2d location = new Translation2d(0,0);
    public ModuleState() {

    }
    public void setTargetRotation(double targetRotation) {
        this.targetRotation = targetRotation;
    }
    public void setTargetRotation(Rotation2d rotation) {
        this.targetRotation = rotation.getDegrees();
    }
    public void setWheelPower(double wheelPower) {
        this.wheelPower = wheelPower;
    }
    public void setLocation(Translation2d location) {
        this.location = location;
    }
    public double getTargetRotation() {
        return targetRotation;
    }
    public double getWheelPower() {
        return wheelPower;
    }
    public ModuleState average(ModuleState averageWith) {
        ModuleState state = new ModuleState();
        state.setTargetRotation((targetRotation+ averageWith.getTargetRotation())/2);
        state.setWheelPower((wheelPower+averageWith.getWheelPower())/2);
        return state;
    }
    public Translation2d getLocation() {
        return location;
    }
    public Translation2d getRotatedLocation(Rotation2d robotRot) {
        return location.rotateBy(robotRot);
    }

}
