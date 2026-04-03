package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class ServoTest extends OpMode {
    TestBenchServo servo = new TestBenchServo();

    @Override
    public void init() {
        servo.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setServoPos(0.0);
        } else {
            servo.setServoPos(1.0);
        }
    }
}
