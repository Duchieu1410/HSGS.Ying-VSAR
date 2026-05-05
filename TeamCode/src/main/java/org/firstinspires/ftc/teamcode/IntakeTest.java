package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ContinuousMotor;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@TeleOp
public class IntakeTest extends OpMode {
    ContinuousMotor intake = new ContinuousMotor();

    @Override
    public void init() {
        intake.init(hardwareMap, "intake_motor", true, 1f);
    }

    @Override
    public void loop() {
        intake.runMotor();
    }
}
