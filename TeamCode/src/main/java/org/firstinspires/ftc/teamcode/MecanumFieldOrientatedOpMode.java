package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

import javax.crypto.ExemptionMechanismException;

@TeleOp
public class MecanumFieldOrientatedOpMode extends OpMode {
    double forward, strafe, rotate;
    MecanumDrive drive = new MecanumDrive();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_down) {
            drive.maxSpeed = Math.max(0.0, drive.maxSpeed - 0.01);
        } else if (gamepad1.dpad_up) {
            drive.maxSpeed = Math.min(1.0, drive.maxSpeed + 0.01);
        }

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        telemetry.addData("forward = ", forward);
        telemetry.addData("strafe = ", strafe);
        telemetry.addData("Max speed = ", drive.maxSpeed);

        drive.driveFieldRelative(forward, strafe, rotate);
    }
}
