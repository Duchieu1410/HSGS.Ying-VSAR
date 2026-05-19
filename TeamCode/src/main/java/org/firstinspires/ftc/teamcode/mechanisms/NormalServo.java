package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NormalServo {

    private Servo servoPos;

    int currentPos = 0;

    public void init(HardwareMap hwMap, String deviceName) {
        servoPos = hwMap.get(Servo.class, deviceName);
        servoPos.setPosition(0);
        currentPos = 0;
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    public void flipState() {
        if (currentPos == 0) {
            setServoPos(0.5);
            currentPos = 1;
        } else {
            setServoPos(0);
            currentPos = 0;
        }
    }


}