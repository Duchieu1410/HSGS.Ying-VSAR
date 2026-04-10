package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeControl {
    public DcMotor outtakeMotor1;
    public double dampenMovement = 0.7;
    final double maxElapsedTime = 10.0;

    // 0.7 - 0.5 is okay
    final double LIFT_POWER = 1;
    final double LOWER_POWER = 0.75;
    public double maxPower = 0.5;

    // ── PID gains ── tweak these on the robot ──────────────────────────
    // kP: main driving force toward target. raise if too slow, lower if oscillating
    // kI: corrects steady-state error (gravity hold). keep small to avoid windup
    // kD: damps overshoot. raise if it oscillates around target
    // TOLERANCE: deadband in ticks — motor shuts off inside this window
    private static double kP = 0.018;
    private static double kI = 0.0008;
    private static double kD = 0;
    private static final int LIFT_TOLERANCE_L = 0, LIFT_TOLERANCE_R = 50;
    private static final int LOWER_TOLERANCE_L = -5, LOWER_TOLERANCE_R = 5;
    public static int    TOLERANCE_L, TOLERANCE_R;      // ticks
    // ──────────────────────────────────────────────────────────────────

    private int    targetPosition  = 0;
    private double integralSum     = 0;
    private double lastError       = 0;
    private boolean pidActive      = false;

    private final ElapsedTime pidTimer  = new ElapsedTime();
    public  final ElapsedTime liftTimer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        outtakeMotor1 = hwMap.get(DcMotor.class, "fr_motor");
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        kD = 0;
//        kP = 0;
//        kI = 0;
    }

    // ── Call this every loop() iteration when in POSITIONAL state ─────
    public void updatePID() {
        if (!pidActive) return;

        double dt    = pidTimer.seconds();
        pidTimer.reset();

        int    currentPos = outtakeMotor1.getCurrentPosition();
        double error      = targetPosition - currentPos;

        // Inside tolerance → stop and hold with brake
        if (TOLERANCE_L <= error && error <= TOLERANCE_R) {
            outtakeMotor1.setPower(0);   // BRAKE behaviour keeps it still
            integralSum = 0;
            lastError   = 0;
            return;
        }

        // Clamp integral to prevent windup (max ±50 ticks worth)
        integralSum += error * dt;
        integralSum  = Math.max(-50 / kI, Math.min(50 / kI, integralSum));

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double power = kP * error
                + kI * integralSum
                + kD * derivative;

        // Clamp output to safe range
        power = Math.max(-1.0, Math.min(1.0, power));

        if (Math.abs(error) <= 30) { // Lower max power when closing in on
            maxPower = 0.15;
        }

        if (power < 0) {
            power = Math.max(power, -maxPower);
        } else {
            power = Math.min(power, maxPower);
        }

        outtakeMotor1.setPower(power);
    }

    // ── Set a new target; PID loop takes over from here ───────────────
    public void moveOuttakeTo(int position) {
        if (position == 0) {
            TOLERANCE_L = LOWER_TOLERANCE_L;
            TOLERANCE_R = LOWER_TOLERANCE_R;
            maxPower = LOWER_POWER;
        } else {
            TOLERANCE_L = LIFT_TOLERANCE_L;
            TOLERANCE_R = LIFT_TOLERANCE_R;
            maxPower = LIFT_POWER;
        }
        targetPosition = position;
        integralSum    = 0;
        lastError      = 0;
        pidActive      = true;
        liftTimer.reset();
        pidTimer.reset();
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int  getTargetPosition()  { return targetPosition; }
    public boolean isPidActive()     { return pidActive; }

    // ── Joystick control (disables PID) ───────────────────────────────
    public void moveOuttakeJoystick(double power) {
        pidActive = false;
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor1.setPower(power * dampenMovement);
    }

    // ── Utilities ─────────────────────────────────────────────────────
    public void resetEncoder() {
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPosition = 0;
        integralSum    = 0;
        lastError      = 0;
    }

    public boolean checkElapsedTime() {
        return liftTimer.seconds() > maxElapsedTime;
    }

    public void resetMotor() {
        pidActive = false;
        outtakeMotor1.setPower(0);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}