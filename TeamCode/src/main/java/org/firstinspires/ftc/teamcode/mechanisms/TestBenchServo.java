package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestBenchServo {

    private Servo servoPos;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "s_pos");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
}
