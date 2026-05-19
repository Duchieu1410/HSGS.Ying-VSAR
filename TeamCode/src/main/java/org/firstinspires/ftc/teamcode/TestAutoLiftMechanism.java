package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ContinuousMotor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.NormalServo;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class TestAutoLiftMechanism extends OpMode {

    ContinuousMotor intake = new ContinuousMotor();
    double forward, strafe, rotate;
    MecanumDrive drive = new MecanumDrive();

    TestBenchServo boxServo = new TestBenchServo();
    NormalServo blockServo = new NormalServo();
    boolean prevL = false;
    boolean prevR = false;
    float prevLT = 0, prevRT = 0;

    OuttakeControl out = new OuttakeControl();

    static final int POS_BASE = 0;   // lowered / intake
    static final int POS_UP   = 15;   // raised / travel
    static final int POS_LP   = 100;
    static final int POS_MP   = 150;
    static final int POS_HP   = 250;

    static final int encoderRatio1 = 1, encoderRatio2 = 5;

    boolean prevA = false, prevB = false, prevX = false, prevY = false;
    boolean prevDpadLeft = false, prevDpadDown = false,
            prevDpadUp   = false, prevDpadRight = false;

    private enum OUTSTATE { JOYSTICK, POSITIONAL }
    private OUTSTATE currentState;

    // Auto lift/lower control for driving
    private enum STATE {
        BASE,
        UP,
        TARGET
    }

    private STATE liftState = STATE.BASE;
    private final ElapsedTime moveStopTimer = new ElapsedTime();
    private static final double DRIVE_DEADBAND = 0.06;
    private static final double STOP_SETTLE_MS = 1000   ;

    public void flipState() {
        currentState = (currentState == OUTSTATE.POSITIONAL)
                ? OUTSTATE.JOYSTICK : OUTSTATE.POSITIONAL;
    }

    public void init() {
        drive.init(hardwareMap);
        out.init(hardwareMap, "right_outtake", "left_outtake", false, true, encoderRatio1, encoderRatio2);
        currentState = OUTSTATE.POSITIONAL;
        boxServo.init(hardwareMap, "s_pos", 0.25);
        blockServo.init(hardwareMap, "block_servo");
        intake.init(hardwareMap, "intake_motor", true, 1);

        out.resetEncoder();
        liftState = STATE.BASE;
        moveStopTimer.reset();
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

        boolean wantsToDrive =
                Math.abs(forward) > DRIVE_DEADBAND ||
                        Math.abs(strafe)  > DRIVE_DEADBAND ||
                        Math.abs(rotate)  > DRIVE_DEADBAND;

        if (gamepad1.dpad_left && !prevDpadLeft)  flipState();
        if (gamepad1.dpad_right && !prevDpadRight) out.resetEncoder();

        // Auto lift/lower based on motion
        if (wantsToDrive) {
            moveStopTimer.reset();
        }
        if (liftState == STATE.BASE && wantsToDrive) {
            out.moveOuttakeTo(POS_UP);
            liftState = STATE.UP;
        }
        if (liftState == STATE.UP && moveStopTimer.milliseconds() >= STOP_SETTLE_MS) {
            out.moveOuttakeTo(POS_BASE);
            liftState = STATE.BASE;
        }

        if (currentState == OUTSTATE.POSITIONAL) {
            if (gamepad1.a && !prevA) {
                out.moveOuttakeTo(POS_LP);
                liftState = STATE.TARGET;
            }
            if (gamepad1.b && !prevB) {
                out.moveOuttakeTo(POS_MP);
                liftState = STATE.TARGET;
            }
            if (gamepad1.x && !prevX) {
                out.moveOuttakeTo(POS_HP);
                liftState = STATE.TARGET;
            }
            if (gamepad1.y && !prevY) {
                out.moveOuttakeTo(POS_UP);
                liftState = STATE.UP;
            }
            if (gamepad1.dpad_down && !prevDpadDown) out.moveOuttakeTo(POS_BASE);

            out.updatePID();

            int pos1 = out.outtakeMotor1.getCurrentPosition() / encoderRatio1;
            int pos2 = out.outtakeMotor2.getCurrentPosition() / encoderRatio2;

            telemetry.addData("Target", out.getTargetPosition());
            telemetry.addData("Pos motor1", pos1);
            telemetry.addData("Pos motor2", pos2);
            telemetry.addData("Skew", pos1 - pos2);
            telemetry.addData("Error", out.getTargetPosition() - (pos1 + pos2) / 2.0);
        }
//        else {
//            double joystick_y = -gamepad1.right_stick_y;
//            if (gamepad1.dpad_up && !prevDpadUp)   out.dampenMovement += 0.02;
//            if (gamepad1.dpad_down && !prevDpadDown) out.dampenMovement -= 0.02;
//
//            out.moveOuttakeJoystick(joystick_y);
//
//            telemetry.addData("Joystick Y", joystick_y);
//            telemetry.addData("Dampen", out.dampenMovement);
//        }

        out.updatePID();

        drive.driveFieldRelative(forward, strafe, rotate);

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("Max speed", drive.maxSpeed);
        telemetry.addData("Lift state", liftState);

        telemetry.addData("State", currentState);

        prevA         = gamepad1.a;
        prevB         = gamepad1.b;
        prevX         = gamepad1.x;
        prevY         = gamepad1.y;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadDown  = gamepad1.dpad_down;
        prevDpadUp    = gamepad1.dpad_up;
        prevDpadRight = gamepad1.dpad_right;

        // These are still your separate mechanisms
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