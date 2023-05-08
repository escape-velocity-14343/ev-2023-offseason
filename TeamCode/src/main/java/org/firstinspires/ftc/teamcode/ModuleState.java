package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class ModuleState {
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
