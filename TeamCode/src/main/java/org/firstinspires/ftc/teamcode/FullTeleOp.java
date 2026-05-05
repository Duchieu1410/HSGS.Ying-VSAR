package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ContinuousMotor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import org.firstinspires.ftc.teamcode.mechanisms.NormalServo;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class FullTeleOp extends OpMode {

    ContinuousMotor intake = new ContinuousMotor();
    double forward, strafe, rotate;
    MecanumDrive drive = new MecanumDrive();

    TestBenchServo boxServo = new TestBenchServo();
    NormalServo blockServo = new NormalServo();
    boolean prevL = false;
    boolean prevR = false;
    float prevLT = 0, prevRT = 0;

    OuttakeControl out = new OuttakeControl();   // single instance, two motors inside

    static final int POS_BASE = 0;
    static final int POS_UP = 15;
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

    public void init() {
        drive.init(hardwareMap);
        out.init(hardwareMap, "right_outtake", "left_outtake", false, true, encoderRatio1, encoderRatio2);
        currentState = OUTSTATE.POSITIONAL;
        boxServo.init(hardwareMap, "s_pos");
        blockServo.init(hardwareMap, "block_servo");
        intake.init(hardwareMap, "intake_motor", true, 1);
    }

    public void loop() {
        intake.runMotor();
        if (gamepad1.dpad_down) {
            drive.maxSpeed = Math.max(0.0, drive.maxSpeed - 0.01);
        } else if (gamepad1.dpad_up) {
            drive.maxSpeed = Math.min(1.0, drive.maxSpeed + 0.01);
        }

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        telemetry.addData("forward = ", forward);
        telemetry.addData("strafe = ", strafe);
        telemetry.addData("Max speed = ", drive.maxSpeed);

        drive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.dpad_left  && !prevDpadLeft)  flipState();
        if (gamepad1.dpad_right && !prevDpadRight)  out.resetEncoder();

        // ── State machine ─────────────────────────────────────────────
        if (currentState == OUTSTATE.POSITIONAL) {
            if (gamepad1.a && !prevA) out.moveOuttakeTo(POS_LP);
            if (gamepad1.b && !prevB) out.moveOuttakeTo(POS_MP);
            if (gamepad1.x && !prevX) out.moveOuttakeTo(POS_HP);
            if (gamepad1.y && !prevY) out.moveOuttakeTo(POS_BASE);
            if (gamepad1.dpad_up && !prevDpadUp) out.moveOuttakeTo(POS_UP);

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

        if (!prevL && gamepad1.left_bumper) {
            boxServo.setServoPos(0.25);
        }
        if (!prevR && gamepad1.right_bumper) {
            boxServo.setServoPos(0.75);
        }

        if (prevRT == 0 && gamepad1.right_trigger > 0) {
            blockServo.setServoPos(0.5);
        }
        if (prevLT == 0 && gamepad1.left_trigger > 0) {
            blockServo.setServoPos(0);
        }

        boxServo.update();

        prevL = gamepad1.left_bumper;
        prevR = gamepad1.right_bumper;
        prevRT = gamepad1.right_trigger;
        prevLT = gamepad1.left_trigger;
    }
}
