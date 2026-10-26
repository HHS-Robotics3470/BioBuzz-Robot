package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake implements Subsystems {
    public static final double FLYWHEEL_POWER = 0.65 ;

    // If rotation is not clockwise, change FORWARD to REVERSE.
    public static final DcMotorSimple.Direction MOTOR_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    private DcMotor flywheel;
    private boolean flywheelOn;

    @Override
    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "turretMotor");
        flywheel.setPower(0);
        flywheel.setDirection(MOTOR_DIRECTION);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelOn = false;
    }

    public void enableFlywheel() {
        flywheelOn = true;
        flywheel.setPower(FLYWHEEL_POWER);
    }

    public void disableFlywheel() {
        flywheelOn = false;
        flywheel.setPower(0);
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public double getPower() {
        return flywheel.getPower();
    }

    @Override
    public void stop() {
        disableFlywheel();
    }
}