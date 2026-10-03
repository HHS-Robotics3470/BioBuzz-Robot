package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Compression {
    public static final String HARDWARE_NAME = "compression";
    public static final String SECOND_HARDWARE_NAME = "compression2";

    // Adjust these to the linkage's safe endpoints.
    public static double RETRACTED_POSITION = 0.25;
    public static double EXTENDED_POSITION = 0.40;

    private Servo servo;
    private double position;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, HARDWARE_NAME);

        servo.setDirection(Servo.Direction.FORWARD);

        retract();
    }

    public void retract() {
        setPosition(RETRACTED_POSITION);
    }

    public void extend() {
        setPosition(EXTENDED_POSITION);
    }

    private void setPosition(double requestedPosition) {
        position = Range.clip(requestedPosition, 0.0, 1.0);

        servo.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}