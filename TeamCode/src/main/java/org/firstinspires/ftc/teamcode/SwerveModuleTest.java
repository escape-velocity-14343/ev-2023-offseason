package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name="Michale VSauce Here!")
public class SwerveModuleTest extends HardwareTemplate {

    public static double p = 0.003;
    public static double i = 0;
    public static double d = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (!isStopRequested()) {
            module.setPid(p, i, d);
            double target = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y));
            double movePower = (gamepad1.left_stick_x*gamepad1.left_stick_x)+(gamepad1.left_stick_y*gamepad1.left_stick_y);
            module.podPid(movePower, target);
            telemetry.update();
        }
    }
}
