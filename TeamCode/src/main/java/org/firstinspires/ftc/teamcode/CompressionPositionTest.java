package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Compression Position Test", group = "Test")
public class CompressionPositionTest extends OpMode {
    private static final double STEP = 0.005;

    private Servo compression;
    private double position = 0.0;

    private boolean previousUp;
    private boolean previousDown;

    @Override
    public void init() {
        compression = hardwareMap.get(Servo.class, "compression");
        compression.setDirection(Servo.Direction.FORWARD);

        position = 0.7;
        compression.setPosition(position);

        previousUp = false;
        previousDown = false;

        telemetry.addLine("Compression initialized at 0.000");
        telemetry.addLine("Controller 2 D-pad UP/DOWN adjusts position");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        if (up && !previousUp && !down) {
            position += STEP;
        } else if (down && !previousDown && !up) {
            position -= STEP;
        }

        position = Range.clip(position, 0.0, 1.0);
        compression.setPosition(position);

        previousUp = up;
        previousDown = down;

        telemetry.addData("Commanded position", "%.3f", position);
        telemetry.addData("Step per press", "%.3f", STEP);
        telemetry.addLine("Record released and compressed positions");
        telemetry.update();
    }
}