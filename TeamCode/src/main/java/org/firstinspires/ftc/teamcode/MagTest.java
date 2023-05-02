package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class MagTest extends LinearOpMode {
    public static double offset = -329;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AnalogInput as5600 = hardwareMap.get(AnalogInput.class, "magnet");
        while (!isStopRequested()) {
            double rawAngle = offset+as5600.getVoltage()*360/3.3;
            telemetry.addData("pos",rawAngle);
            telemetry.addData("warp", AngleUnit.normalizeDegrees(rawAngle));
            telemetry.update();
        }
    }
}
