package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import java.util.concurrent.locks.Lock;

/**
 * Created by CAD1 on 12/8/2016.
 */
@Autonomous(name = "pixytest", group = "4507")

public class PixyTest extends LinearOpMode {

    I2cDevice pixy;
    I2cDeviceReader pixyReader;
    I2cAddr pixyADDR = I2cAddr.create8bit(0x54);
    int pixyRegister = 0x50;
    byte pixyData[];
    Lock pixyReadLock;


    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDevice.get("pixy");
        pixyReader = new I2cDeviceReader(pixy, pixyADDR, pixyRegister, 6);

        waitForStart();
        pixy.enableI2cReadMode(pixyADDR, 0x50, 6);
        while (!pixy.isI2cPortReady()) {
            idle();
        }
        pixyReadLock = pixy.getI2cReadCacheLock();
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
        Log.i("pixyData", pixyData.toString());
        telemetry.addData("byte0", pixyData[1]);
        telemetry.addData("byte1", pixyData[2]);
        telemetry.addData("byte2", pixyData[3]);
        telemetry.addData("byte3", pixyData[4]);
        telemetry.addData("byte4", pixyData[5]);
        telemetry.addData("byte5", pixyData[6]);
        updateTelemetry(telemetry);

        while(!isStopRequested()) {
            idle();
        }

    }


}
