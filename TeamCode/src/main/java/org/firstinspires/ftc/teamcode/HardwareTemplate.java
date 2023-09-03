package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public abstract class HardwareTemplate extends LinearOpMode {
    public Motor ltop,lbottom,rtop,rbottom;
    public AS5600 lrot,rrot;
    public SwerveModule lmodule, rmodule;
    public IMU imu;
    VoltageSensor voltageSensor;
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ltop = new Motor(hardwareMap,"ltm");
        lbottom = new Motor(hardwareMap,"lbm");
        lrot = new AS5600(hardwareMap,"lrot");

        rtop = new Motor(hardwareMap,"rtm");
        rbottom = new Motor(hardwareMap,"rbm");
        rrot = new AS5600(hardwareMap,"rrot");

        lmodule = new SwerveModule(ltop, lbottom, lrot, telemetry);
        rmodule = new SwerveModule(rtop, rbottom, rrot, telemetry);
    }
    public static boolean compare(double a, double b, double tolerance) {
        return (Math.abs(a-b)<tolerance);
    }

}
