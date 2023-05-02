package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class TestMotor extends LinearOpMode {
    //public static double power = 0.5;
    public static double p = 0.0025;
    public static double i = 0.19;
    public static double d = 0.0007;
    public static double offset = -329;
    public static double f = 0.2;

    public static int target = 0;
    double power = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorImplEx motorEx = (DcMotorImplEx) hardwareMap.get(DcMotor.class,"motor");
        AnalogInput as5600 = hardwareMap.get(AnalogInput.class, "magnet");
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        Motor motor = new Motor(hardwareMap,"motor");
       // motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.resetEncoder();
        PIDController controller = new PIDController(p,i,d);
        controller.setTolerance(3,1);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while (!isStopRequested()) {
            double angle = AngleUnit.normalizeDegrees(offset+as5600.getVoltage()*360/3.3);
            power = controller.calculate(angle,target) + f*Math.cos(Math.toRadians(angle));
            if (!controller.atSetPoint())
                motor.set(power);
            else
                motor.set(f*Math.cos(Math.toRadians(angle)));
            controller.setPID(p,i,d);
            telemetry.addData("p",p);
            telemetry.addData("target",target);
            telemetry.addData("error",controller.getPositionError());
            telemetry.addData("power",power);
            telemetry.addData("position",angle);
            telemetry.addData("speed",motor.getCorrectedVelocity());
            telemetry.addData("amps",motorEx.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }
    public static boolean compare(double a, double b, double tolerance) {
        return (Math.abs(a-b)<tolerance);
    }
}
