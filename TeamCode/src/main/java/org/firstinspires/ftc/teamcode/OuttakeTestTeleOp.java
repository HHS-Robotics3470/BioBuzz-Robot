package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.outtake;

@TeleOp(name = "Outtake Test", group = "Test")
public class OuttakeTestTeleOp extends OpMode {

    private outtake outtake;

    @Override
    public void init() {
        outtake = new outtake();
        outtake.init(hardwareMap);
        telemetry.addLine("R2: latch full power ON | L2: OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // L2 wins if both triggers are pressed.
        if (gamepad1.left_trigger > 0.1) {
            outtake.disableFlywheel();
        } else if (gamepad1.right_trigger > 0.1) {
            outtake.enableFlywheel();
        }

        // Runs at 1.0 until L2 or OpMode Stop, even after R2 is released.
        outtake.updateFlywheel();

        telemetry.addData("Flywheel", outtake.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Commanded power", outtake.isFlywheelOn() ? org.firstinspires.ftc.teamcode.subSystems.outtake.MOTOR_POWER : 0.0);
        telemetry.addData("Motor 1 velocity", "%.0f ticks/s", outtake.getMotor1Velocity());
        telemetry.addData("Motor 2 velocity", "%.0f ticks/s", outtake.getMotor2Velocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (outtake != null) outtake.stop();
    }
}
