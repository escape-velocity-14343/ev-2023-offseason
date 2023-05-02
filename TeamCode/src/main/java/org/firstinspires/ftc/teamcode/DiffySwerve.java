package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class DiffySwerve extends LinearOpMode {

    public static double offset = 0;
    public static double topPower = 0;
    public static double bottomPower = 0;
    public static double p = 0.008;
    public static double i = 0.05;
    public static double d = 0.001;
    public static double target = 0;
    public double kP = 1.5;
    public double kI = 0.01;
    public double kD = 0.001;


    ElapsedTime time = new ElapsedTime();
    double lastTime=0,lastRot=0,rotation=0,rotspeed=0;
    Motor top,bottom;
    //BNO055IMUImpl;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        top = new Motor(hardwareMap,"tm");
        bottom = new Motor(hardwareMap,"bm");
        AS5600 rot = new AS5600(hardwareMap,"rot");

        top.setRunMode(Motor.RunMode.RawPower);

        bottom.setRunMode(Motor.RunMode.RawPower);
        time.reset();

        PIDController pid = new PIDController(p,i,d);

        while (!isStopRequested()) {
            rotation = rot.getDegrees()+offset;
            rotspeed = AngleUnit.normalizeDegrees(rotation-lastRot)/(time.seconds()-lastTime);
            pid.setPID(p,i,d);
            pid.setTolerance(0.1);
            pid.setSetPoint(AngleUnit.normalizeDegrees(target-rotation));
            if (!pid.atSetPoint()) {
                podMove(topPower, -pid.calculate());
            }
            else {
                podMove(topPower,0);
            }
            //podMove(topPower,bottomPower);
            if (kP!=0) {
                top.setVeloCoefficients(kP,kI,kD);
                bottom.setVeloCoefficients(kP,kI,kD);
            }
            telemetry.addData("p", top.getVeloCoefficients()[0]);
            telemetry.addData("i", top.getVeloCoefficients()[1]);
            telemetry.addData("d", top.getVeloCoefficients()[2]);
            telemetry.addData("voltage", rot.analog.getVoltage());
            telemetry.addData("target",target);
            telemetry.addData("rot", rotation);
            telemetry.addData("rotSpeed", rotspeed);
            telemetry.addData("time",time.seconds()-lastTime);
            telemetry.addData("rotdelta", rotation-lastRot);
            telemetry.addData("topv",top.getCorrectedVelocity());
            telemetry.addData("bottomv",bottom.getCorrectedVelocity());
            telemetry.update();
            lastTime = time.seconds();
            lastRot = rotation;



        }
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
