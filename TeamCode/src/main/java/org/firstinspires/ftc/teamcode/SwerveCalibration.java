package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class SwerveCalibration extends HardwareTemplate {

    public static double p = 0.02;
    public static double i = 0;
    public static double d = 0.001;
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        double movePower;
        while (!isStopRequested()) {
            lmodule.setPid(p, i, d);
            rmodule.setPid(p,i,d);
            //target = Math.toDegrees(Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y));
            movePower=0;
            if (lmodule.close(target)&&rmodule.close(target)) {
                movePower = (gamepad1.left_stick_x*gamepad1.left_stick_x)+(gamepad1.left_stick_y*gamepad1.left_stick_y);
            }
            lmodule.podPid(movePower, target);
            rmodule.podPid(movePower, target);
            telemetry.update();
        }
    }
}
