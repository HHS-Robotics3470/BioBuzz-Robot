package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Rotation Test", group = "Test")
public class TurretRotationTest extends OpMode {
    private static final double CENTER = 0.5;

    // Expand gradually after testing mechanical clearance.
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    private static final double DEADBAND = 0.08;
    private static final double POSITION_PER_SECOND = 0.10;

    // Change to -1.0 if stick right turns the turret left.
    private static final double STICK_DIRECTION = 1.0;

    private Servo rRotation;
    private Servo lRotation;

    private double targetPosition = CENTER;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        rRotation = hardwareMap.get(Servo.class, "rRotation");
        lRotation = hardwareMap.get(Servo.class, "lRotation");

        // Both shafts should turn in the same direction.
        rRotation.setDirection(Servo.Direction.FORWARD);
        lRotation.setDirection(Servo.Direction.FORWARD);

        targetPosition = CENTER;
        applyPosition();

        telemetry.addLine("Both servos commanded to center: 0.500");
        telemetry.addLine("Align gears before testing linked movement");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        double stick = gamepad2.left_stick_x;
        double input = 0.0;

        if (Math.abs(stick) > DEADBAND) {
            input = Math.signum(stick)
                    * (Math.abs(stick) - DEADBAND)
                    / (1.0 - DEADBAND);
        }

        targetPosition += input * STICK_DIRECTION
                * POSITION_PER_SECOND * dt;

        targetPosition = Range.clip(
                targetPosition, MIN_POSITION, MAX_POSITION
        );

        applyPosition();

        telemetry.addData("Target position", "%.3f", targetPosition);
        telemetry.addData("Right command", "%.3f", rRotation.getPosition());
        telemetry.addData("Left command", "%.3f", lRotation.getPosition());
        telemetry.addData("Minimum", "%.3f", MIN_POSITION);
        telemetry.addData("Maximum", "%.3f", MAX_POSITION);
        telemetry.addLine("Controller 2 left stick: turn");
        telemetry.addLine("Release stick: hold target");
        telemetry.addLine("Positions are commands, not measured angles");
        telemetry.update();
    }

    private void applyPosition() {
        rRotation.setPosition(targetPosition);
        lRotation.setPosition(targetPosition);
    }
}