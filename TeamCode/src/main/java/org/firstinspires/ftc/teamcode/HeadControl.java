package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.NormalServo;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class HeadControl extends OpMode {
    TestBenchServo boxServo = new TestBenchServo();
    NormalServo blockServo = new NormalServo();
    boolean prevL = false;
    boolean prevR = false;
    float prevLT = 0, prevRT = 0;
    int counter = 0;

    @Override
    public void init() {
        boxServo.init(hardwareMap, "s_pos");
        blockServo.init(hardwareMap, "block_servo");
    }

    @Override
    public void loop() {
        telemetry.addData("Right trigger: ", gamepad1.right_trigger);
        telemetry.addData("Left trigger: ", gamepad1.left_trigger);
        telemetry.addData("Counter: ", counter);
        if (!prevL && gamepad1.left_bumper) {
            boxServo.setServoPos(0.25);
        }
        if (!prevR && gamepad1.right_bumper) {
            boxServo.setServoPos(0.75);
        }

        if (prevRT == 0.0 && gamepad1.right_trigger > 0) {
            blockServo.setServoPos(0.5);
            counter += 1;
        }
        if (prevLT == 0.00 && gamepad1.left_trigger > 0) {
            blockServo.setServoPos(0);
        }

        boxServo.update();

        prevL = gamepad1.left_bumper;
        prevR = gamepad1.right_bumper;
        prevRT = gamepad1.right_trigger;
        prevLT = gamepad1.left_trigger;
    }
}
