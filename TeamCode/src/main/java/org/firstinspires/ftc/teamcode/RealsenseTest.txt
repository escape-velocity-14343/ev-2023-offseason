package org.firstinspires.ftc.teamcode;

import static org.opencv.core.CvType.CV_16UC1;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.CvType.CV_8UC3;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;
import android.widget.TextView;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.DepthFrame;
import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.Option;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.PipelineProfile;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.Sensor;
import com.intel.realsense.librealsense.StreamType;
import com.intel.realsense.librealsense.VideoFrame;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
@TeleOp
public class RealsenseTest extends LinearOpMode{
    RsContext rsContext;
    float centerDepth =0;
    boolean ded = true;
    private Continuation<? extends Consumer<Bitmap>> bitmapContinuation;
    Bitmap bitmapFromMat;
    byte[] imageBytes;

    private final Object bitmapFrameLock = new Object();
    Pipeline pipe = new Pipeline();
    Size intelSize = new Size(240,360);


    @Override
    public void runOpMode() {
        PutMatPipeline put = new PutMatPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                //FtcDashboard.getInstance().startCameraStream(cam,15);
            }
            @Override
            public void onError(int errorCode)
            {
                Log.println(Log.ERROR,"beleln","opencv goofed");
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        cam.setPipeline(put);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RsContext.init(hardwareMap.appContext);
        rsContext=new RsContext();

        rsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {

                ded = false;
                try(PipelineProfile pp = pipe.start()){
                }
                catch(Exception e) {
                    Log.println(Log.ERROR,"WOAHHH",""+e);
                }
            }

            @Override
            public void onDeviceDetach() {
                ded = true;
            }
        });

        //waitForStart();



        // try statement needed here to release resources allocated by the Pipeline:start() method
        try(PipelineProfile pp = pipe.start()){
        }
        catch(Exception e) {
            Log.println(Log.ERROR,"OHHHH",""+e);
        }
        if (rsContext.queryDevices().getDeviceCount()>0) {
            ded=false;
        }

        while (!isStopRequested()) {
            if (!ded) {
                try (FrameSet frames = pipe.waitForFrames()) {
                    try (Frame f = frames.first(StreamType.DEPTH))
                    {
                        DepthFrame depth = f.as(Extension.DEPTH_FRAME);
                        intelSize = new Size(depth.getWidth(),depth.getHeight());

                        put.setMat(DepthFrame2Mat(depth));

                        centerDepth = depth.getDistance(depth.getWidth()/2, depth.getHeight()/2);

                        Log.println(Log.INFO,"goofy",""+centerDepth);
                    }
                }
                catch (Exception e) {
                    Log.println(Log.ERROR,"AHHH",""+e);

                }
            }

            telemetry.addLine("hi");
            telemetry.addData("depth",centerDepth);
            telemetry.update();
            /*synchronized (bitmapFrameLock) {
                bitmapContinuation.dispatch(new ContinuationResult<Consumer<Bitmap>>()
                {
                    @Override
                    public void handle(Consumer<Bitmap> bitmapConsumer)
                    {
                        if (imageBytes!=null);
                        bitmapFromMat = getIntelBitmap();
                        bitmapConsumer.accept(bitmapFromMat);
                        bitmapFromMat.recycle();
                    }
                });
                bitmapContinuation = null;
            }*/

        }
        rsContext.close();
        rsContext.removeDevicesChangedCallback();
        pipe.stop();
        pipe.close();
    }
    //https://stackoverflow.com/questions/71381082/converting-16-bit-depth-frame-from-intel-realsense-d455-to-opencv-mat-in-android
    public static Mat DepthFrame2Mat(final DepthFrame frame) {
        Mat frameMat = new Mat(frame.getHeight(), frame.getWidth(), CV_8UC1);
        final int bufferSize = (int)(frameMat.total() * frameMat.elemSize());
        byte[] dataBuffer = new byte[bufferSize];
        frame.getData(dataBuffer);
        ByteBuffer.wrap(dataBuffer).order(ByteOrder.LITTLE_ENDIAN).asReadOnlyBuffer().get(dataBuffer);
        frameMat.put(0,0, dataBuffer);
        return frameMat;
    }
    /*public static Mat DepthFrame2Mat(final DepthFrame frame) {
        Mat frameMat = new Mat(frame.getHeight(), frame.getWidth(), CV_8UC1);
        final int bufferSize = (int)(frameMat.total() * frameMat.elemSize());
        byte[] dataBuffer = new byte[bufferSize];
        short[] s = new short[dataBuffer.length / 2];
        frame.getData(dataBuffer);
        ByteBuffer.wrap(dataBuffer).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(s);

        frameMat.put(0,0, s );
        return frameMat;
    }*/
    /*@Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation)
    {
        synchronized (bitmapFrameLock)
        {
            bitmapContinuation = continuation;
        }
    }
    public Bitmap getIntelBitmap() {
        int size = (int) (intelSize.height * intelSize.width);
        int[] pixelData = new int[size];
        for (int i = 0; i < size; i++) {
            // pack 3 bytes into int for ARGB_8888
            int r = imageBytes[3 * i];
            int g = imageBytes[3 * i + 1];
            int b = imageBytes[3 * i + 2];
            pixelData[i] = Color.rgb(r, g, b);
        }
        Bitmap map = Bitmap.createBitmap(pixelData,
                (int) intelSize.width,
                (int) intelSize.height,
                Bitmap.Config.ARGB_8888);
        return Bitmap.createBitmap(map, 0, 0, (int) intelSize.width, (int) intelSize.height);
    }*/
}
