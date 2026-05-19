package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestBenchServo {

    private Servo servoPos;
    private double currentPos = 0;
    private double targetPos = 0;
    private double startPos = 0;
    private long moveStartTime = -1;
    private long moveDurationMs = 1500;

    public void init(HardwareMap hwMap, String deviceName, double origin) {
        servoPos = hwMap.get(Servo.class, deviceName);
        servoPos.setPosition(origin);
        currentPos = origin;
        targetPos = origin;
    }

    public void setServoPos(double target) {
        if (target == targetPos) return;
        startPos = currentPos;
        targetPos = target;
        moveStartTime = System.currentTimeMillis();
    }

    public void setMoveDuration(long ms) {
        moveDurationMs = ms;
    }

    public void update() {
        if (moveStartTime < 0) return;

        long elapsed = System.currentTimeMillis() - moveStartTime;
        if (elapsed >= moveDurationMs) {
            currentPos = targetPos;
            moveStartTime = -1;
        } else {
            double t = (double) elapsed / moveDurationMs;
            currentPos = startPos + t * (targetPos - startPos);
        }
        servoPos.setPosition(currentPos);
    }

    public double getServoPos() {
        return servoPos.getPosition();
    }
}
