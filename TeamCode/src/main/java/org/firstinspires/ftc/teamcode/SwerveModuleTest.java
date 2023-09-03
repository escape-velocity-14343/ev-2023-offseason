package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class SwerveModuleTest extends HardwareTemplate {

    public static double p = 0.01;
    public static double i = 0;
    public static double d = 0.001;
    double lt,rt;
    double jx,x;
    double jy,y;
    double rot;
    double lx,ly,rx,ry;

    double[] powers = {0,0,0,0};

    Translation2d translation = new Translation2d();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (!isStopRequested()) {
            lmodule.setPid(p*12.5/voltageSensor.getVoltage(),i,d);
            rmodule.setPid(p*12.5/voltageSensor.getVoltage(),i,d);

            /*rotation = gamepad1.right_stick_x;
            movePower = (gamepad1.left_stick_x*gamepad1.left_stick_x)+(gamepad1.left_stick_y*gamepad1.left_stick_y);*/
            rot = -gamepad1.right_stick_x/2.0;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            translation = new Translation2d(jx,jy);
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
             jx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
             jy = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            powers = new double[]{jx, jy + rot, jx, jy - rot};

            lx = normalize(powers, 1)[0];
            ly = normalize(powers, 1)[1];
            rx = normalize(powers, 1)[2];
            ry = normalize(powers, 1)[3];
            lt = Math.atan2(ly,lx);
            rt = Math.atan2(ry,rx);
            lt = Math.toDegrees(lt);
            rt = Math.toDegrees(rt);
            telemetry.addData("lx", lx);
            telemetry.addData("ly", ly);
            telemetry.addData("lt",lt);
            telemetry.addData("yaw", botHeading);


            if (!compare(x+y+rot,0,0.001)) {
                lmodule.podPidXY(lx,ly);
                rmodule.podPidXY(rx,ry);
            }
            else {
                lmodule.podPid(0, lrot.getDegrees());
                rmodule.podPid(0, rrot.getDegrees());
            }


            if ((lmodule.close(lt)&&rmodule.close(rt))) {

            }
            else {

            }
            telemetry.update();
            if (gamepad1.options) {
                imu.resetYaw();
            }
        }
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

}
