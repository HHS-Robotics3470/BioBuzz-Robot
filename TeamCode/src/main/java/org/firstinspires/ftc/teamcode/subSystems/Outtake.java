package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outtake {
    public static final String MOTOR_NAME = "outake";
    public static final String HOOD_NAME = "hoodServo";

    public static double FLYWHEEL_POWER = 1.0;
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 0.55;
    public static double HOOD_STEP = 0.005;

    private DcMotor flywheel;
    private Servo hood;
    private boolean flywheelOn;
    private double hoodPosition;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
        flywheelOn = false;

        hoodPosition = Range.clip(HOOD_MIN, 0.0, 1.0);
        hood.setPosition(hoodPosition);
    }

    public void enableFlywheel() {
        flywheelOn = true;
        flywheel.setPower(Range.clip(FLYWHEEL_POWER, 0.0, 1.0));
    }

    public void disableFlywheel() {
        flywheelOn = false;
        flywheel.setPower(0.0);
    }

    public void moveHoodUp() {
        setHoodPosition(hoodPosition + HOOD_STEP);
    }

    public void moveHoodDown() {
        setHoodPosition(hoodPosition - HOOD_STEP);
    }

    public void setHoodPosition(double requestedPosition) {
        double min = Range.clip(HOOD_MIN, 0.0, 1.0);
        double max = Range.clip(HOOD_MAX, min, 1.0);
        hoodPosition = Range.clip(requestedPosition, min, max);
        hood.setPosition(hoodPosition);
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void stop() {
        disableFlywheel();
    }
}