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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 4507 Robocahrgers on 9/10/16.
 */

@Autonomous(name="Autonomous", group="4507")  // @TeleOp(...) is the other common choice
//@Disabled
public class Autonomous4507 extends LinearOpMode {

    // DcMotorController
    DcMotor l1;
    DcMotor l2;
    DcMotor r1;
    DcMotor r2;
    // Servos
//    Servo beaconPinion;
//    Servo beaconPusher;
//    // Sensors
    GyroSensor gyro;
//    ColorSensor bColor;
//    ColorSensor fColor;
//    UltrasonicSensor beaconU;
//    //Switches
//    DigitalChannel tileSw;
//    DigitalChannel colorSw;
//    DigitalChannel beaconSw;
//    // Elapsed Time
//    ElapsedTime elapsedTime;

    // Global State Vaiables
    double countsPerYard = 30000.0;
    double countsPer360 = 30000.0;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    long fastSpDelay;
    long slowSpDelay;
    int speedToCounts[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9};

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
        // DcMotorControllers
        l1 = hardwareMap.dcMotor.get("l1");
//        l1.setDirection(DcMotorSimple.Direction.REVERSE);
        l2 = hardwareMap.dcMotor.get("l2");
        r1 = hardwareMap.dcMotor.get("r1");
        r2 = hardwareMap.dcMotor.get("r2");
//        r2.setDirection(DcMotorSimple.Direction.REVERSE);
        // Servos
//        beaconPinion = hardwareMap.servo.get("bPi");
//        beaconPusher = hardwareMap.servo.get("bPu");
//        beaconPinion.setPosition(0.5);
//        beaconPusher.setPosition(0.5);
//        // Sensors
        gyro = hardwareMap.gyroSensor.get("gyro");
//        bColor = hardwareMap.colorSensor.get("bC");
//        fColor = hardwareMap.colorSensor.get("fC");
//        beaconU = hardwareMap.ultrasonicSensor.get("bU");
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

        // Wait for the robot to start
        waitForStart();

        /**
         * Moves and Descriptions
         * 1.
         *
         * 2.
         *
         * 3.
         *
         * 4.
         *
         * 5.
         *
         * 6.
         *
         * 7.
         *
         * 8.
         *
         */

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
        driveStraight(12, 1.0, 5000);
        driveTurn(90, 5000);
        driveStraight(12, 1.0, 5000);






    }


    /**
     * Created on 9/10/16
     * @param inches is how far to go
     * @param speed is how fast to go
     * @param delayMillis is how long to delay
     * @throws InterruptedException
     */

    public void driveStraight(double inches, double speed, long delayMillis) throws InterruptedException {
        int lTarget;
        int rTarget;

        lTarget = l1.getCurrentPosition() + (int)(inches * (countsPerYard / 36.0));
        rTarget = r1.getCurrentPosition() + (int)(inches * (countsPerYard / 36.0));

        setMotorPosition(lTarget, rTarget, speed);
        sleep(delayMillis);
    }

    public void driveTurn (int degrees, long delay) throws InterruptedException {
        int currentHeading;
        int startHeading;
        int desiredHeading;
        int gyroError;
        boolean exitTurn = false;

        if (blue) {
            degrees = degrees * -1;
        }

        startHeading = gyro.getHeading();
        desiredHeading = gyro.getHeading() + degrees;
        if (desiredHeading > 360) {
            desiredHeading = desiredHeading - 360;
        }
        if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
        }
        gyroError = desiredHeading - gyro.getHeading();
        if (gyroError > 180) {
            gyroError = 360 - gyroError;
        }
        if (gyroError < -180) {
            gyroError = 360 + gyroError;
        }



        while (!exitTurn) {
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
            l1.setPower(Range.clip(0 - (gyroError * 0.05), -1.0, 1.0));
            l2.setPower(Range.clip(0 - (gyroError * 0.05), -1.0, 1.0));
            r1.setPower(Range.clip(0 + (gyroError * 0.05), -1.0, 1.0));
            r2.setPower(Range.clip(0 + (gyroError * 0.05), -1.0, 1.0));
            idle();
            sleep(delay);
        }
    }

    /**
     *
     * @param shooterSpeed
     * @param moverSpeed
     */

    public void runUmbrella (double shooterSpeed, double moverSpeed) {

    }

//    public void pushButton () {
//        while (beaconU.getUltrasonicLevel() > beaconDist) {
//            beaconPinion.setPosition(1.0);
//        }
//
//    }

    public void setMotorPosition (int lTarget, int rTarget, double speed) throws InterruptedException {
        int preEncoderPos = l1.getCurrentPosition();
        int b4EncoderPos;
        int speedUpDist;
        int direction = 1;
        if (speed < 0) {
            direction = -1;
        }
        for(int c = 0; c < (int)speed * 100; c++) {
            b4EncoderPos = l1.getCurrentPosition();
            l1.setPower((double)c / 100.0 * direction);
            l2.setPower((double)c / 100.0 * direction);
            r1.setPower((double)c / 100.0 * direction);
            r2.setPower((double)c / 100.0 * direction);
            while (l1.getCurrentPosition() < b4EncoderPos + speedToCounts[c]) {
                idle();
            }
        }
        speedUpDist = l1.getCurrentPosition() - preEncoderPos;
        while (lTarget - speedUpDist > l1.getCurrentPosition()) {
            idle();
        }
        for(int c = (int)speed * 100; c <= 0; c--) {
            b4EncoderPos = l1.getCurrentPosition();
            l1.setPower((double)c / 100.0 * direction);
            l2.setPower((double)c / 100.0 * direction);
            r1.setPower((double)c / 100.0 * direction);
            r2.setPower((double)c / 100.0 * direction);
            while (l1.getCurrentPosition() < b4EncoderPos + speedToCounts[c]) {
                idle();
            }
        }
    }

}
