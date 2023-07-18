package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
public class VISIONTEST extends LinearOpMode {

    public static double multiplier = 1/640d;
    public static double xM = 1/640d;
    public static double yM = 1/640d;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        PipelineKiwi kiwi = new PipelineKiwi();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(kiwi);
                kiwi.setColor(PipelineKiwi.DetectType.BLUE);
            }
            @Override
            public void onError(int errorCode)
            {
                Log.println(Log.ERROR,"code","OPENCV GOOFED!!!");

            }
        });
        Triangulator triangulator = new Triangulator();
        triangulator.setCameraAngle(Math.toRadians(-60));
        triangulator.setCameraHeight(10);
        waitForStart();
        while (opModeIsActive()) {
            int cx = kiwi.getX()-320;
            int cy = 240-kiwi.getY();
            telemetry.addData("camera x", cx);
            telemetry.addData("camera y", cy);
            telemetry.addData("camera w", kiwi.getWidth());
            triangulator.triangulateFromPixels(cx,cy,kiwi.getWidth());
            telemetry.addData("x", triangulator.getX());
            telemetry.addData("y", triangulator.getY());
            telemetry.addData("z", triangulator.getZ());
            /*telemetry.addData("object x", Math.sin(Math.toRadians(horizangle))*distance);d
            telemetry.addData("object y", Math.cos(Math.toRadians(horizangle))*distance);
            telemetry.addData("object y2", Math.sin(Math.toRadians(verticangle))*distance);
            telemetry.addData("object z", Math.cos(Math.toRadians(verticangle))*distance);*/

            telemetry.update();
        }

    }
}
