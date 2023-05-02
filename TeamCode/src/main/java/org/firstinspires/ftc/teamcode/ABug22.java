package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ABug22 extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    double beforeTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.a) {
                beforeTime = time.milliseconds();
            }
            if (time.milliseconds()-beforeTime>500&&beforeTime!=-1) {
                //do something
                beforeTime = -1;
            }
        }

    }
}
