package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeControl {
    public DcMotor outtakeMotor1;   // e.g. right
    public DcMotor outtakeMotor2;   // e.g. left

    public double dampenMovement = 0.15;
    private int encoderRatio1 = 1, encoderRatio2 = 1;

    private static final double LIFT_POWER  = 0.35;
    private static final double LOWER_POWER = 0.2;
    private static final double NEAR_TARGET_POWER = 0.15;

    private static final double kP    = 0.018;
    private static final double kI    = 0.0008;
    private static final double kD    = 0.0000;
    private static final double kSYNC = 0.01;

    private static final int POSITION_TOLERANCE = 10;   // target error tolerance
    private static final int SYNC_TOLERANCE = 2;        // motor-to-motor mismatch tolerance
    private static final double MAX_INTEGRAL = 300.0;   // anti-windup
    private static final double MAX_SYNC_CORRECTION = 0.30;

    private int targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private boolean pidActive = false;
    public boolean reachedTarget = true;

    private double currentMaxPower = LIFT_POWER;

    private final ElapsedTime pidTimer  = new ElapsedTime();
    public final ElapsedTime liftTimer = new ElapsedTime();

    public void init(HardwareMap hwMap,
                     String motor1Name,
                     String motor2Name,
                     boolean reverse1,
                     boolean reverse2,
                     int ratio1,
                     int ratio2) {

        outtakeMotor1 = hwMap.get(DcMotor.class, motor1Name);
        outtakeMotor2 = hwMap.get(DcMotor.class, motor2Name);

        encoderRatio1 = Math.max(1, ratio1);
        encoderRatio2 = Math.max(1, ratio2);

        for (DcMotor m : new DcMotor[]{outtakeMotor1, outtakeMotor2}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        outtakeMotor1.setDirection(reverse1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(reverse2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private boolean checkError(double error) {
        if (targetPosition == 0) {
            if (error >= -2 && error <= 10) {
                return true;
            } else {
                return false;
            }
        } else {
            return Math.abs(error) <= POSITION_TOLERANCE;
        }
    }

    public void updatePID() {
        if (!pidActive || outtakeMotor1 == null || outtakeMotor2 == null) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt < 0.005) dt = 0.005;

        int pos1 = outtakeMotor1.getCurrentPosition() / encoderRatio1;
        int pos2 = outtakeMotor2.getCurrentPosition() / encoderRatio2;

        double avgPos = (pos1 + pos2) / 2.0;
        double error = targetPosition - avgPos;
        double skewError = pos1 - pos2;

        // Stop when both the average target and the motor mismatch are small
        if (checkError(error) && Math.abs(skewError) <= SYNC_TOLERANCE) {
            stopMotors();
            integralSum = 0;
            lastError = 0;
            reachedTarget = true;
            return;
        }

        // PID on the average position
        integralSum += error * dt;
        integralSum = clamp(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double basePower = kP * error + kI * integralSum + kD * derivative;

        // Use lower output near the target, but do not permanently modify the field
        double maxOutput = currentMaxPower;
        if (Math.abs(error) <= 18) {
            maxOutput = Math.min(maxOutput, NEAR_TARGET_POWER);
        }

        basePower = clamp(basePower, -maxOutput, maxOutput);

        // Sync correction: keep the motors aligned with each other
        double correction = clamp(kSYNC * skewError, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);

        // If one motor is mounted the opposite way, swap the signs here.
        double power1 = clamp(basePower - correction, -maxOutput, maxOutput);
        double power2 = clamp(basePower + correction, -maxOutput, maxOutput);

        reachedTarget = false;
        outtakeMotor1.setPower(power1);
        outtakeMotor2.setPower(power2);
    }

    public void moveOuttakeTo(int position) {
        currentMaxPower = (position == 0) ? LOWER_POWER : LIFT_POWER;

        targetPosition = position;
        integralSum = 0;
        lastError = 0;
        pidActive = true;

        liftTimer.reset();
        pidTimer.reset();

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveOuttakeJoystick(double power) {
        pidActive = false;
        integralSum = 0;
        lastError = 0;

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double out = power * dampenMovement;
        outtakeMotor1.setPower(out);
        outtakeMotor2.setPower(out);
    }

    public void resetEncoder() {
        for (DcMotor m : new DcMotor[]{outtakeMotor1, outtakeMotor2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        targetPosition = 0;
        integralSum = 0;
        lastError = 0;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public boolean isPidActive() {
        return pidActive;
    }

    public void resetMotor() {
        pidActive = false;
        stopMotors();
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}