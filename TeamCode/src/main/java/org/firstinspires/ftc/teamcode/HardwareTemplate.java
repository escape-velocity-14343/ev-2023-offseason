package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class HardwareTemplate extends LinearOpMode {
    public Motor top,bottom;
    public AS5600 rot;
    public SwerveModule module;
    public void initialize() {
        top = new Motor(hardwareMap,"tm");
        bottom = new Motor(hardwareMap,"bm");
        rot = new AS5600(hardwareMap,"rot");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        module = new SwerveModule(top, bottom, rot, telemetry);
    }
}
