package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Husk extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HuskyLens husky = hardwareMap.get(HuskyLens.class,"huskylens");

       while (!isStopRequested()) {
           husky.sendCommandRead(HuskyLens.Command.COMMAND_REQUEST);
           telemetry.addData("hi",husky.getConnectionInfo());
           telemetry.update();
           sleep(1);
       }
    }
}
