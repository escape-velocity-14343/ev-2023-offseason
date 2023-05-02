package org.firstinspires.ftc.teamcode;

import android.renderscript.RenderScript;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;
import java.util.List;

@I2cDeviceType
@DeviceProperties(name = "DFRobot HuskyLens Camera", xmlTag = "Huskylens")


public class HuskyLens extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x32);
    Command lastCommand = Command.COMMAND_NONE;

    public HuskyLens(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        //this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        Log.println(Log.INFO,"husky","HuskyLens Initialized");
        Log.println(Log.INFO, "husky engaged", ""+this.deviceClient.isEngaged());
    }

    public enum Register {
        FIRST(0),
        HEADER(0x55),
        HEADER2(0xAA),
        ADDRESS(0x11),
        DATAL(0x0A),
        COMMAND(0x2A),
        DATA0(0x2C),
        DATA1(0x01),
        DATA2(0xC8),
        DATA3(0x00),
        DATA4(0x0A),
        DATA5(0x00),
        DATA6(0x14),
        DATA7(0x00),
        DATA8(0x01),
        DATA9(0x00),
        CHECKSUM(0x58),
        LAST(CHECKSUM.bVal);
        public int bVal;
        Register(int bVal) {
            this.bVal = bVal;
        }
    }
    public enum Command {
        COMMAND_NONE(0x00),
        COMMAND_REQUEST_KNOCK(0x2C),
        COMMAND_RETURN_OK(0x2E),
        COMMAND_REQUEST(0x20);
        public int bVal;
        Command(int bVal) {
            this.bVal = bVal;
        }
    }


    protected void writeShort(final Register reg, short value)
    {

        deviceClient.write(12, TypeConversion.shortToByteArray(value));
        Log.println(Log.DEBUG,"husky",Integer.toHexString(value&0xFF));
    }




    protected short readShort(Register reg)
    {
        //return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));

        return deviceClient.read8(reg.bVal);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return (knock());
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "DFRobot HuskyLens Camera";
    }
    public void sendCommand(Command command, byte[] data) {
        List<Byte> bytes = new ArrayList<>();
        int cs = 0;
        bytes.add((byte) 85);
        cs+=85;
        bytes.add((byte) 170);
        cs+=170;
        bytes.add((byte) 17);
        cs+=17;
        bytes.add((byte) 0);
        bytes.add((byte) command.bVal);
        cs+=command.bVal;
        cs = cs%256;
        bytes.add((byte) cs);
        byte[] beets = new byte[bytes.size()];


        for (int i = 0; i<bytes.size(); i++) {
            beets[i] = bytes.get(i);
            Log.println(Log.DEBUG, "husy", ""+Integer.toHexString(bytes.get(i)&0xFF));
        }

        deviceClient.write(12,beets);
        this.lastCommand = command;

        /*if (lastRegister == Register.FIRST) {
            writeByte(Register.HEADER, (short) Register.HEADER.bVal);
            lastRegister = Register.HEADER;
        }
        if (lastRegister == Register.HEADER) {
            writeByte(Register.HEADER2, (short) Register.HEADER2.bVal);
            lastRegister = Register.HEADER2;
        }
        if (lastRegister == Register.HEADER2) {
            writeByte(Register.ADDRESS, (short) Register.ADDRESS.bVal);
            lastRegister = Register.ADDRESS;
        }
        if (lastRegister == Register.ADDRESS) {
            writeByte(Register.DATAL, (short) 0x00);
            lastRegister = Register.DATAL;
        }
        if (lastRegister == Register.DATAL) {
            writeByte(Register.COMMAND, (short) command.bVal);
            lastRegister = Register.COMMAND;
        }
        if (lastRegister == Register.COMMAND) {
            writeByte(Register.CHECKSUM, (short) 0x3C);
            lastRegister = Register.FIRST;
        }*/
        /*

        w

        */
    }
    public void sendCommand(Command command) {
        sendCommand(command, null);
    }
    public void sendComm(Command command) {

    }
    public boolean knock() {
        return sendCommandRead(Command.COMMAND_REQUEST_KNOCK);
    }
    public boolean sendCommandRead(Command command) {
        sendCommand(command);

        List<Short> outputs = new ArrayList<>();
        int j = 6;
        for (int i = 0; i < j ; i++) {
            outputs.add((short) deviceClient.read8());
            Log.println(Log.INFO,"husky",Integer.toHexString(outputs.get(i)&0xFF));
            if (i==3&&outputs.get(3)>0) {
                j+=outputs.get(3);
            }
        }
        //data will be from 3 to 3+j

        if (lastCommand == Command.COMMAND_REQUEST&&j>6) {
            int blocks = outputs.get(5);
            List<Translation2d> values = new ArrayList<>();
            for (int i = 0; i<blocks; i++) {
                for (int k = 0; k<6; k++) {

                    deviceClient.read8();
                }
                int x = deviceClient.read8();
                x += 256*deviceClient.read8();
                int y = deviceClient.read8();
                y += 256*deviceClient.read8();
                values.add(new Translation2d(x,y));
            }
            for (int k = 0; k<8; k++) {

                deviceClient.read8();
            }
            for (Translation2d translation : values) {
                Log.println(Log.VERBOSE, "huskyData", "x"+translation.getX());
                Log.println(Log.VERBOSE, "huskyData", "y"+translation.getY());
            }

        }


        if (outputs.get(4)==Command.COMMAND_RETURN_OK.bVal) {
            return true;
        }
        else
            return false;
    }

}
