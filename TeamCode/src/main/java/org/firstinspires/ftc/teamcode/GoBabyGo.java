package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Go Baby Go!")
public class GoBabyGo extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;

    public void init() {
        leftDrive = hardwareMap.dcMotor.get("lD");
        rightDrive = hardwareMap.dcMotor.get("rD");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        leftDrive.setPower(-gamepad1.left_stick_y);
        rightDrive.setPower(-gamepad1.right_stick_y);
    }
}
