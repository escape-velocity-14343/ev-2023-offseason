package org.firstinspires.ftc.teamcode;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PutMatPipeline extends OpenCvPipeline {
    Mat mat;
    @Override
    public Mat processFrame(Mat input) {
        if (mat == null)
            return input;
        //Imgproc.cvtColor(mat,mat, CvType.CV_8UC1);]
        Size size = input.size();
        mat.convertTo(input,CvType.CV_8UC1);
        Imgproc.resize(input,input,size);


        return input;
    }
    public void setMat(Mat input) {
        mat = input;
    }
}
