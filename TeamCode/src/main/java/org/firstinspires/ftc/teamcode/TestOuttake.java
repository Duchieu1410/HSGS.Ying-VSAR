package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeControl;

@TeleOp
public class TestOuttake extends OpMode {
    OuttakeControl out = new OuttakeControl();   // single instance, two motors inside

    static final int POS_BASE = 0;
    static final int POS_LP   = 100;
    static final int POS_MP   = 150;
    static final int POS_HP   = 250;

    static final int encoderRatio1 = 1, encoderRatio2 = 5;

    boolean prevA = false, prevB = false, prevX = false, prevY = false;
    boolean prevDpadLeft = false, prevDpadDown = false,
            prevDpadUp   = false, prevDpadRight = false;

    private enum OUTSTATE { JOYSTICK, POSITIONAL }
    private OUTSTATE currentState;

    public void flipState() {
        currentState = (currentState == OUTSTATE.POSITIONAL)
                ? OUTSTATE.JOYSTICK : OUTSTATE.POSITIONAL;
    }

    @Override
    public void init() {
        out.init(hardwareMap, "right_outtake", "left_outtake", false, true, encoderRatio1, encoderRatio2);
        currentState = OUTSTATE.POSITIONAL;
    }

    @Override
    public void loop() {
        // ── Mode switching ────────────────────────────────────────────
        if (gamepad1.dpad_left  && !prevDpadLeft)  flipState();
        if (gamepad1.dpad_right && !prevDpadRight)  out.resetEncoder();

        // ── State machine ─────────────────────────────────────────────
        if (currentState == OUTSTATE.POSITIONAL) {
            if (gamepad1.a && !prevA) out.moveOuttakeTo(POS_LP);
            if (gamepad1.b && !prevB) out.moveOuttakeTo(POS_MP);
            if (gamepad1.x && !prevX) out.moveOuttakeTo(POS_HP);
            if (gamepad1.y && !prevY) out.moveOuttakeTo(POS_BASE);

            out.updatePID();

            int pos1 = out.outtakeMotor1.getCurrentPosition() / encoderRatio1;
            int pos2 = out.outtakeMotor2.getCurrentPosition() / encoderRatio2;

            telemetry.addData("Target",     out.getTargetPosition());
            telemetry.addData("Pos motor1", pos1);
            telemetry.addData("Pos motor2", pos2);
            telemetry.addData("Skew",       pos1 - pos2);
            telemetry.addData("Error",      out.getTargetPosition() - (pos1 + pos2) / 2.0);
//            telemetry.addData("maxPower",   out.maxPower);

        } else {
            double joystick_y = -gamepad1.right_stick_y;
            if (gamepad1.dpad_up   && !prevDpadUp)   out.dampenMovement += 0.02;
            if (gamepad1.dpad_down && !prevDpadDown)  out.dampenMovement -= 0.02;

            out.moveOuttakeJoystick(joystick_y);

            telemetry.addData("Joystick Y",  joystick_y);
            telemetry.addData("Dampen",      out.dampenMovement);
        }

        telemetry.addData("State",   currentState);
//        telemetry.addData("TOLERANCE_L", out.TOLERANCE_L);

        prevA         = gamepad1.a;
        prevB         = gamepad1.b;
        prevX         = gamepad1.x;
        prevY         = gamepad1.y;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadDown  = gamepad1.dpad_down;
        prevDpadUp    = gamepad1.dpad_up;
        prevDpadRight = gamepad1.dpad_right;
    }
}