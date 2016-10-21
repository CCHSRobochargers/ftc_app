/**
 * Created by 4507 Robochargers on
 *          9/ 10/ 16
 *
 *9/10/16:
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 4507 Robocahrgers on 9/10/16.
 */

@Autonomous(name="Autonomous", group="4507")  // @TeleOp(...) is the other common choice
//@Disabled
public class Autonomous4507 extends LinearOpMode {

    // TODO: Need to test everything new
    // TODO: (Which is practically everything)
    // TODO: Look and fix code at other "TODO:" statements

    // Device Interface Module
    DeviceInterfaceModule dim;
    final int BLUE_LED_CHANNEL = 0;
    final int RED_LED_CHANNEL = 1;
    // DcMotor
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;
    // Servos
    CRServo beaconPusher;
    Servo rangeRotater;
//    // Sensors
    ModernRoboticsI2cGyro gyro;
    ColorSensor bColor;
    float beaconHsvValues[] = {0F,0F,0F};
    ColorSensor fColorf;
    float frontFloorHsvValues[] = {0F,0F,0F};
    ColorSensor fColorb;
    float backFloorHsvValues[] = {0F,0F,0F};
    ModernRoboticsI2cRangeSensor frontRotatingRange;
    public static final I2cAddr NEW_I2C_ADDRESS_FOR_RANGE = I2cAddr.create8bit(0x2A);
    ModernRoboticsI2cRangeSensor backRange;
//    //Switches
//    DigitalChannel tileSw;
//    DigitalChannel colorSw;
//    DigitalChannel beaconSw;
//    // Elapsed Time
//    ElapsedTime elapsedTime;

    // Global State Vaiables
    int countsPerYard = 2867;
    int countsPer4Donuts = 30000;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    long fastSpDelay;
    long slowSpDelay;

    // These variables are for autonomous.
    boolean red;
    boolean blue;
    boolean diagTile;
    boolean straightTile;
    boolean beaconY;
    boolean beaconN;
    boolean endMove = false;

    double move0a;
    double move0b;
    double move1a;
    double move1b;
    double move2;
    double move3;
    double move4a;
    double move4b;
    double move5;
    double move6;

    int turn0a;
    int turn0b;
    int turn1;
    int turn2;
    int turn3;

    // beacon dist must be in cm ****
    double beaconDist;
    boolean redR;
    boolean redL;
    boolean blueR;
    boolean blueL;


    @Override
    public void runOpMode() throws InterruptedException {
        // Device Interface Module
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        dim.setLED(RED_LED_CHANNEL, true);
        dim.setLED(BLUE_LED_CHANNEL, false);
        // DcMotor
        leftDrive = hardwareMap.dcMotor.get("l1");
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive = hardwareMap.dcMotor.get("r1");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kicker = hardwareMap.dcMotor.get("kick");
        kicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sweeper = hardwareMap.dcMotor.get("sweep");
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Servos
        beaconPusher = hardwareMap.crservo.get("bPu");
        beaconPusher.setPower(0.0);
        rangeRotater = hardwareMap.servo.get("rR");
        rangeRotater.setPosition(0.5);
//        // Sensors
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        bColor = hardwareMap.colorSensor.get("bC");
        fColorf = hardwareMap.colorSensor.get("cFF");
        fColorb = hardwareMap.colorSensor.get("cFB");
        frontRotatingRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "fR");
        frontRotatingRange.setI2cAddress(NEW_I2C_ADDRESS_FOR_RANGE); //TODO: Find i2c Address to change it
//                                                                     TODO: Fixed???
        backRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bR");
//        //Switches
//        tileSw = hardwareMap.digitalChannel.get("tSw");
//        colorSw = hardwareMap.digitalChannel.get("cSw");
//        beaconSw = hardwareMap.digitalChannel.get("bSw");
//        if(tileSw.getState()) {
//            diagTile = true;
//            straightTile = false;
//        } else if (!tileSw.getState()) {
//            diagTile = false;
//            straightTile = true;
//        }
//        if(colorSw.getState()) {
//            red = true;
//            blue = false;
//        } else if (!colorSw.getState()) {
//            red = false;
//            blue = true;
//        }
//        if (beaconSw.getState()) {
//            beaconY = true;
//            beaconN = true;
//        } else if (!beaconSw.getState()) {
//            beaconY = false;
//            beaconN = true;
//        }
//        // Elapsed Time
//        elapsedTime = new ElapsedTime();
        gyro.calibrate();
        while (gyro.isCalibrating() && !isStarted()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
            dim.setLED(RED_LED_CHANNEL, false);
            dim.setLED(BLUE_LED_CHANNEL, true);
        }

        // Wait for the robot to start
        waitForStart();


//        if (straightTile) {
//            driveStraight(move1a + move1b, fastSpeed, fastSpDelay);
//        } else if (diagTile) {
//            driveStraight(move0a, fastSpeed, fastSpDelay);
//            driveTurn(turn0a, mediumSpeed, slowSpDelay);
//            driveStraight(move0b, fastSpeed, fastSpDelay);
//            driveTurn(turn0b, mediumSpeed, slowSpDelay);
//            driveStraight(move1b, fastSpeed, fastSpDelay);
//        }
//        if (beaconY) {
//            driveTurn(turn1, mediumSpeed, slowSpDelay);
//            driveStraight(move2, fastSpeed, fastSpDelay);
//            driveTurn(turn2, mediumSpeed, slowSpDelay);
//            driveStraight(move3, slowSpeed,slowSpDelay);
//            // add button code
//            driveStraight(move4a, fastSpeed, fastSpDelay);
//            driveStraight(move4b, slowSpeed, slowSpDelay);
//            // add button code
//            driveStraight(move5, fastSpeed, fastSpDelay);
//            driveTurn(turn3, mediumSpeed, slowSpDelay);
//            driveStraight(move6, fastSpeed, fastSpDelay);
//        }

        while (gyro.isCalibrating() && opModeIsActive()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
            dim.setLED(RED_LED_CHANNEL, false);
            dim.setLED(BLUE_LED_CHANNEL, true);
        }
        sweep(true);
        driveStraight(8, 1.0, 1000);
        shoot(2);
        driveTurn(45, .75, 1000);
        driveStraight(32, 1.0, 1000);
        driveTurn(45, 0.75, 1000);
        driveStraight(22, 1.0, 1000);
        driveTurn(-90, 0.75, 1000);
        driveStraight(60, 1.0, 1000);
        sweep(false);






    }


    /**
     * Created on 9/10/16
     * @param inches is how far to go
     * @param speed is how fast to go
     * @param delayMillis is how long to delay
     * @throws InterruptedException allows the sleep() method to be used
     */

    public void driveStraight(double inches, double speed, long delayMillis) throws InterruptedException {
        int lTarget = leftDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));
        int rTarget = rightDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive()) {
            idle();
        }

        sleep(delayMillis);
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     * @param speed is how fast to turn
     * @param delayMillis is how long to delay
     * @throws InterruptedException allows the sleep() method to be used
     */

    public void driveTurn(int degrees, double speed, long delayMillis) throws InterruptedException {
        if (blue) {
            degrees = degrees * -1;
        }

        driveTurnWithEncoders(degrees, speed);
        driveTurnWithGyro(degrees);

        sleep(delayMillis);
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     * @param speed is how fast to turn
     */

    public void driveTurnWithEncoders(int degrees, double speed) {
        int lTarget = leftDrive.getCurrentPosition() + (degrees * (countsPer4Donuts / 1440));
        int rTarget = rightDrive.getCurrentPosition() - (degrees * (countsPer4Donuts / 1440));

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive()) {
            idle();
        }
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     */

    public void driveTurnWithGyro (int degrees) {
        int desiredHeading;
        int gyroError;
        boolean exitTurn = false;

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        desiredHeading = gyro.getHeading() + degrees;
        if (desiredHeading > 360) {
            desiredHeading = desiredHeading - 360;
        }
        if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
        }

        while (!exitTurn && opModeIsActive()) {
            gyroError = gyro.getHeading() - desiredHeading;
            if (gyroError > 180) {
                gyroError = 360 - gyroError;
            }
            if (gyroError < -180) {
                gyroError = 360 + gyroError;
            }
            if (gyroError == 0) {
                exitTurn = true;
            }
            leftDrive.setPower(Range.clip(0 - (gyroError * 0.03), -0.25, 0.25));
            rightDrive.setPower(Range.clip(0 + (gyroError * 0.03), -0.25, 0.25));
            idle();
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * TODO: This is going to be an absolute pain in the butt
     * @param speed is the speed to go
     * @param colorRed is whether or not we are looking for red
     * @param desiredDistAwayInInches is the distance we want to be from the wall when we attempt to press the button
     */

    public void beacon(double speed, boolean colorRed, int desiredDistAwayInInches) {
        boolean frontLooks = false;
        boolean backLooks = false;
        boolean stop = false;

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Need to figure out which color is which :
        if (!colorRed) {
            speed = -speed;
            backLooks = true;
        } else if (colorRed) {
            frontLooks = true;
        }

        rangeRotater.setPosition(0.5);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (!stop && opModeIsActive()) {

            if (backLooks) {
                Color.RGBToHSV(fColorb.red(), fColorb.green(), fColorb.blue(), backFloorHsvValues);
                if (backFloorHsvValues[0] >= 69.0) {
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                }
            }
        }
    }

    /**
     *
     * @param times is the number of times to shoot
     */

    public void shoot(int times) {
        for (int c = 0; c < times; c++) {
            kick();
            index();
        }
    }

    /**
     * kicks a particle once
     */

    public void kick() {
        kicker.setTargetPosition(kicker.getCurrentPosition() + 1120);
        kicker.setPower(1.0);
        while (kicker.isBusy() && opModeIsActive()) {
            idle();
        }
    }

    /**
     * indexes a particle once
     */

    public void index() {

    }

    /**
     *
     * @param onOff is a boolean to determine whether to stop or start the sweeper
     */

    public void sweep(boolean onOff) {
        if (onOff) {
            sweeper.setPower(1.0);
        } else if (!onOff) {
            sweeper.setPower(0.0);
        }
    }
}
