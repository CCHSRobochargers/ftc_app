package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.concurrent.locks.Lock;

/**
 * Created by CAD1 on 12/8/2016.
 */
@Autonomous(name = "pixytest", group = "4507")

public class PixyTest extends LinearOpMode {

    I2cDevice pixy;
    I2cDeviceSynch pixyReader;
    I2cAddr pixyADDR = I2cAddr.create8bit(0x54);
    int pixyRegister = 0x50;
    byte[] pixyData;// = new byte[] { 1, 2, 3, -1, -2, -3 };
    int pixyDataConverted;
    Lock pixyReadLock;
    int counter = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDevice.get("pixy");
        pixyReader = new I2cDeviceSynchImpl(pixy, pixyADDR, false);
        pixyReader.engage();

        waitForStart();
        while(!isStopRequested()) {

            pixyData = pixyReader.read(pixyRegister, 6);
            pixyDataConverted = ((int) pixyData[0]);
            counter++;
    //        pixy.enableI2cReadMode(pixyADDR, 0x50, 6);
    //        while (!pixy.isI2cPortReady()) {
    //            idle();
    //        }
    //        pixyReadLock = pixy.getI2cReadCacheLock();
    //        pixyReadLock.lock();
    //        pixyData = pixyReader.getReadBuffer();
    //        pixyReadLock.unlock();

    //        pixy.readI2cCacheFromController();
    //
    //        pixyReadLock = pixy.getI2cReadCacheLock();
    //        pixyReadLock.lock();
    //        pixyData = pixy.getI2cReadCache();
    //        pixyReadLock.unlock();
    //        pixy.setI2cPortActionFlag();
            String s = byteArrayToString(pixyData);
            int i = TypeConversion.byteArrayToInt(pixyData, ByteOrder.LITTLE_ENDIAN);
            Log.i("pixyData", pixyData.toString());
            telemetry.addData("byte0", pixyData[0]);
            telemetry.addData("byte1", pixyData[1]);
            telemetry.addData("byte2", pixyData[2]);
            telemetry.addData("byte3", pixyData[3]);
            telemetry.addData("byte4", pixyData[4]);
            telemetry.addData("byte5", pixyData[5]);
            telemetry.addData("pixyData", pixyData);
            telemetry.addData("dataLength", pixyData.length);
            telemetry.addData("s", s);
            telemetry.addData("i", i);
            telemetry.addData("counter", counter);
            updateTelemetry(telemetry);
//            for(int c = 0; c < 6; c++) {
//                pixyData[c] = 0;
//            }
            sleep(100);


            idle();
        }

    }
    String byteArrayToString(byte[] in) {
        char out[] = new char[in.length * 2];
        for (int i = 0; i < in.length; i++) {
            out[i * 2] = "0123456789ABCDEF".charAt((in[i] >> 4) & 15);
            out[i * 2 + 1] = "0123456789ABCDEF".charAt(in[i] & 15);
        }
        return new String(out);
    }


}

