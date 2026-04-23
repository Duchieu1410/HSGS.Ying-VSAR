package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ContinuousMotor {
    private DcMotor motor;
    private int power = 0;

    public void init(HardwareMap hwMap, String motorName, boolean reverseMotor, int motorPower) {
        motor = hwMap.get(DcMotor.class, motorName);

        if (reverseMotor) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        power = motorPower;
    }

    public void modifyPower(int newPower) {
        power = newPower;
    }

    public void reverseDirection() {
        if (motor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void runMotor() {
        motor.setPower(power);
    }
}
