package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.SingleOuttakeControl;

@TeleOp
public class SingleAxisController extends OpMode {
    SingleOuttakeControl out1 = new SingleOuttakeControl();

    static final int POS_BASE = 0;
    static final int POS_MAX  = 1500;
    static final int POS_LP   = 100;
    static final int POS_MP   = 150;
    static final int POS_HP   = 250;

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
        out1.init(hardwareMap, "right_outtake", false);
        currentState = OUTSTATE.POSITIONAL;
    }

    @Override
    public void loop() {
        // ── Timeout safety ────────────────────────────────────────────
//        if (out.checkElapsedTime()) out.resetMotor();

        // ── Mode switching ────────────────────────────────────────────
        if (gamepad1.dpad_left  && !prevDpadLeft)  flipState();
        if (gamepad1.dpad_right && !prevDpadRight) {
            out1.resetEncoder();
        }

        // ── State machine ─────────────────────────────────────────────
        if (currentState == OUTSTATE.POSITIONAL) {
//            if (out.checkElapsedTime()) {
//                out.resetMotor();
//            }

            if (gamepad1.a && !prevA) {
                out1.moveOuttakeTo(POS_LP);
            }
            if (gamepad1.b && !prevB) {
                out1.moveOuttakeTo(POS_MP);
            }
            if (gamepad1.x && !prevX) {
                out1.moveOuttakeTo(POS_HP);
            }
            if (gamepad1.y && !prevY) {
                out1.moveOuttakeTo(POS_BASE);
            }

            // ★ Drive the PID every loop iteration
            out1.updatePID();

            telemetry.addData("Target position",   out1.getTargetPosition());
            telemetry.addData("Current position",  out1.outtakeMotor1.getCurrentPosition());
            telemetry.addData("Error",
                    out1.getTargetPosition() - out1.outtakeMotor1.getCurrentPosition());
            telemetry.addData("maxPower", out1.maxPower);
            telemetry.addData("toleranceL", out1.TOLERANCE_L);

        } else {
            double joystick_y = -gamepad1.right_stick_y;
            if (gamepad1.dpad_up   && !prevDpadUp)   out1.dampenMovement += 0.02;
            if (gamepad1.dpad_down && !prevDpadDown)  out1.dampenMovement -= 0.02;

            out1.moveOuttakeJoystick(joystick_y);

            telemetry.addData("Joystick Y",       joystick_y);
            telemetry.addData("Dampen movement",  out1.dampenMovement);
            telemetry.addData("Power: ", joystick_y * out1.dampenMovement);
        }

        telemetry.addData("State",            currentState);
        telemetry.addData("Encoder position", out1.outtakeMotor1.getCurrentPosition());

        prevA          = gamepad1.a;
        prevB          = gamepad1.b;
        prevX          = gamepad1.x;
        prevY          = gamepad1.y;
        prevDpadLeft   = gamepad1.dpad_left;
        prevDpadDown   = gamepad1.dpad_down;
        prevDpadUp     = gamepad1.dpad_up;
        prevDpadRight  = gamepad1.dpad_right;
    }
}