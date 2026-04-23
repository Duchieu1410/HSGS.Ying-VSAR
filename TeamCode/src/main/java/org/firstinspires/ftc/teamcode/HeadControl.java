package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class HeadControl extends OpMode {
    TestBenchServo boxServo = new TestBenchServo();
    TestBenchServo blockServo = new TestBenchServo();
    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false, prevY = false;

    @Override
    public void init() {
        boxServo.init(hardwareMap, "s_pos");
        blockServo.init(hardwareMap, "block_servo");
    }

    @Override
    public void loop() {
        if (!prevA && gamepad1.a) {
            boxServo.setServoPos(0.25);
        }
        if (!prevB && gamepad1.b) {
            boxServo.setServoPos(0);
        }

        if (!prevX && gamepad1.x) {
            blockServo.setServoPos(0.5);
        }

        if (!prevY && gamepad1.y) {
            blockServo.setServoPos(0);
        }
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }
}
