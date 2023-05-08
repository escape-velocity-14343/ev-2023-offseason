package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class SwerveControl extends LinearOpMode {

    public static double offset = 0;
    public static double topPower = 0;
    public static double bottomPower = 0;
    public static double p = 0.008;
    public static double i = 0;
    public static double d = 0.001;





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
        double target = 0;

        while (!isStopRequested()) {
            target = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y));
            double movePower = (gamepad1.left_stick_x*gamepad1.left_stick_x)+(gamepad1.left_stick_y*gamepad1.left_stick_y);

            rotation = rot.getDegrees()+offset;
            rotspeed = AngleUnit.normalizeDegrees(rotation-lastRot)/(time.seconds()-lastTime);
            double moveTo = AngleUnit.normalizeDegrees(target-rotation);
            if (!compare(moveTo,0,90)) {
                moveTo = AngleUnit.normalizeDegrees(moveTo-180);
                movePower*=-1;
            }
            pid.setPID(p,i,d);
            pid.setTolerance(1);
            pid.setSetPoint(moveTo);


            podMove(Math.cos(Math.toRadians(moveTo))*movePower, pid.calculate());


            telemetry.addData("p", top.getVeloCoefficients()[0]);
            telemetry.addData("i", top.getVeloCoefficients()[1]);
            telemetry.addData("d", top.getVeloCoefficients()[2]);
            telemetry.addData("move", movePower);
            telemetry.addData("voltage", rot.analog.getVoltage());
            telemetry.addData("target",AngleUnit.normalizeDegrees(target));
            telemetry.addData("rot", AngleUnit.normalizeDegrees(rotation));
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
    public static boolean compare(double a, double b, double tolerance) {
        return (Math.abs(a-b)<tolerance);
    }
}
