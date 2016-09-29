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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4507 Robocahrgers on 9/10/16.
 */

@Autonomous(name="Autonomous", group="4507")  // @TeleOp(...) is the other common choice
@Disabled
public class Autonomous4507 extends LinearOpMode {

    DcMotorController dcmc;
    //Motors
    DcMotor lDrive;
    DcMotor rDrive;
    // Servos
    Servo beaconPinion;
    Servo beaconPusher;
    // Sensors
    GyroSensor gyro;
    ColorSensor bColor;
    ColorSensor fColor;
    UltrasonicSensor beaconU;
    //Switches
    DigitalChannel tileSw;
    DigitalChannel colorSw;
    DigitalChannel beaconSw;
    // Elapsed Time
    ElapsedTime elapsedTime;

    // Global State Vaiables
    double countsPerYard = 30000.0;
    double countsPer360 = 30000.0;
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
        // DcMotors
        lDrive = hardwareMap.dcMotor.get("lD");
        rDrive = hardwareMap.dcMotor.get("rD");
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Servos
        beaconPinion = hardwareMap.servo.get("bPi");
        beaconPusher = hardwareMap.servo.get("bPu");
        beaconPinion.setPosition(0.5);
        beaconPusher.setPosition(0.5);
        // Sensors
        gyro = hardwareMap.gyroSensor.get("gyro");
        bColor = hardwareMap.colorSensor.get("bC");
        fColor = hardwareMap.colorSensor.get("fC");
        beaconU = hardwareMap.ultrasonicSensor.get("bU");
        //Switches
        tileSw = hardwareMap.digitalChannel.get("tSw");
        colorSw = hardwareMap.digitalChannel.get("cSw");
        beaconSw = hardwareMap.digitalChannel.get("bSw");
        if(tileSw.getState()) {
            diagTile = true;
            straightTile = false;
        } else if (!tileSw.getState()) {
            diagTile = false;
            straightTile = true;
        }
        if(colorSw.getState()) {
            red = true;
            blue = false;
        } else if (!colorSw.getState()) {
            red = false;
            blue = true;
        }
        if (beaconSw.getState()) {
            beaconY = true;
            beaconN = true;
        } else if (!beaconSw.getState()) {
            beaconY = false;
            beaconN = true;
        }
        // Elapsed Time
        elapsedTime = new ElapsedTime();

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

        if (straightTile) {
            driveStraight(move1a + move1b, fastSpeed, fastSpDelay);
        } else if (diagTile) {
            driveStraight(move0a, fastSpeed, fastSpDelay);
            driveTurn(turn0a, mediumSpeed, slowSpDelay);
            driveStraight(move0b, fastSpeed, fastSpDelay);
            driveTurn(turn0b, mediumSpeed, slowSpDelay);
            driveStraight(move1b, fastSpeed, fastSpDelay);
        }
        if (beaconY) {
            driveTurn(turn1, mediumSpeed, slowSpDelay);
            driveStraight(move2, fastSpeed, fastSpDelay);
            driveTurn(turn2, mediumSpeed, slowSpDelay);
            driveStraight(move3, slowSpeed,slowSpDelay);
            // add button code
            driveStraight(move4a, fastSpeed, fastSpDelay);
            driveStraight(move4b, slowSpeed, slowSpDelay);
            // add button code
            driveStraight(move5, fastSpeed, fastSpDelay);
            driveTurn(turn3, mediumSpeed, slowSpDelay);
            driveStraight(move6, fastSpeed, fastSpDelay);
        }
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

        lTarget = lDrive.getCurrentPosition() + (int)(inches * (countsPerYard / 36.0));
        rTarget = rDrive.getCurrentPosition() + (int)(inches * (countsPerYard / 36.0));

        lDrive.setTargetPosition(lTarget);
        rDrive.setTargetPosition(rTarget);

        lDrive.setPower(speed);
        rDrive.setPower(speed);

        while (!lDrive.isBusy() || !rDrive.isBusy()) {
            // Empty While loop runs until motors start
        }

        while (lDrive.isBusy() || rDrive.isBusy()) {
            // Empty While loop runs until motors stop
        }

        sleep(delayMillis);
    }

    public void driveTurn (int degrees, double speed, long delay) throws InterruptedException {
        int currentHeading;
        int startHeading;
        int desiredHeading;
        int gyroError;
        boolean exitTurn = false;

        if (blue) {
            degrees = degrees * -1;
        }

        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        desiredHeading = gyro.getHeading();
        startHeading = gyro.getHeading();
        gyroError = desiredHeading - gyro.getHeading();
        if (gyroError > 180) {
            gyroError = 360 - gyroError;
        }
        if (gyroError < -180) {
            gyroError = 360 + gyroError;
        }

        desiredHeading = gyro.getHeading() + degrees;
        if (desiredHeading > 360) {
            desiredHeading = desiredHeading - 360;
        }
        if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
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
            idle();
        }
        sleep(delay);
    }

    /**
     *
     * @param shooterSpeed
     * @param moverSpeed
     */

    public void runUmbrella (double shooterSpeed, double moverSpeed) {

    }

    public void pushButton () {
        while (beaconU.getUltrasonicLevel() > beaconDist) {
            beaconPinion.setPosition(1.0);
        }

    }

}
