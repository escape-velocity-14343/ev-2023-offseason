package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config

public class AsymmetricMotionProfile extends LinearOpMode {
    public static double start = 1;
    public static double end = 3;
    InterpLUT lut = new InterpLUT();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lut.add(0,0);
        lut.add(start,1);
        lut.add(2,1);
        lut.add(end,0);
        lut.add(10,0);
        lut.createLUT();
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        double position = 0;
        int multiplier = 1;
        while (!isStopRequested()) {
            if (elapsedTime.seconds()>4) {
                elapsedTime.reset();
                multiplier*=-1;
            }
            position = position+0.00005*lut.get(elapsedTime.seconds())*multiplier;
            telemetry.addData("power",lut.get(elapsedTime.seconds())*multiplier);
            telemetry.addData("position",position);
            telemetry.update();
        }


    }
}
