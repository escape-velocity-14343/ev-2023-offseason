package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AS5600{
    AnalogInput analog;
    /**
     * Constructs the instance AS5600 for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */

    public AS5600(HardwareMap hMap, String id) {
        analog = hMap.get(AnalogInput.class, id);

    }
    public double getDegrees() {
        double v = analog.getVoltage();
        double voltage = 0;
        if (v!=0) {
            voltage = 3.3-(3.3*(3.3+2*v)-3.3*Math.sqrt(3.3*3.3-4*3.3*v+12*v*v))/(4*v);
        }

        return v*360/5.0;
    }
    public double getRadians() {
        return Math.toRadians(getDegrees());
    }

}
