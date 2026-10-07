package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.Compression;
import org.firstinspires.ftc.teamcode.subSystems.Outtake;

@TeleOp(name = "TeleOp Red", group = "Test")
public class TeleOPRed extends OpMode {
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP_INTERVAL_SECONDS = 0.04;

    private Outtake shooter;
    private Compression compression;
    private Mecnum drive;
    private final ElapsedTime hoodTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter = new Outtake();
        shooter.init(hardwareMap);

        compression = new Compression();
        compression.init(hardwareMap);

        hoodTimer.reset();
        telemetry.addLine("SHOOTER COMPRESSION TEST v3 loaded");
        telemetry.update();

        drive = new Mecnum(); drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            shooter.disableFlywheel();
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            shooter.enableFlywheel();
        }

        if (hoodTimer.seconds() >= HOOD_STEP_INTERVAL_SECONDS) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                shooter.moveHoodUp();
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                shooter.moveHoodDown();
            }
            hoodTimer.reset();
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            compression.retract();
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            compression.extend();
        }

        drive.driveRobot(gamepad1);

        telemetry.addLine("SHOOTER COMPRESSION TEST v3");
        telemetry.addData("Left bumper", gamepad1.left_bumper);
        telemetry.addData("Right bumper", gamepad1.right_bumper);
        telemetry.addData("Flywheel", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Hood position", "%.3f", shooter.getHoodPosition());
        telemetry.addData("Compression position", "%.3f", compression.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
    }
}