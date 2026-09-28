package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class outtake implements Subsystems {

    // Match these strings to the names in the Robot Configuration.
    public static String MOTOR_1_NAME = "outake";
    public static String MOTOR_2_NAME = "outake2";
    public static double MOTOR_POWER = 1.0;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private boolean flywheelOn = false;

    @Override
    public void init(HardwareMap hw) {
        motor1 = hw.get(DcMotorEx.class, MOTOR_1_NAME);
        motor2 = hw.get(DcMotorEx.class, MOTOR_2_NAME);

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheelOn = false;
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    public void enableFlywheel() {
        flywheelOn = true;
    }

    public void disableFlywheel() {
        flywheelOn = false;
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    public void updateFlywheel() {
        double power = flywheelOn ? MOTOR_POWER : 0.0;
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public double getMotor1Velocity() {
        return motor1.getVelocity();
    }

    public double getMotor2Velocity() {
        return motor2.getVelocity();
    }
    @Override
    public void stop() {
        disableFlywheel();
    }
 }