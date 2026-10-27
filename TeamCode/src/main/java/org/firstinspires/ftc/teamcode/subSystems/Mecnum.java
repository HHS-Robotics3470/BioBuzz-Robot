
package org.firstinspires.ftc.teamcode.subSystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.RobotHardware;


public class Mecnum implements Subsystems {
    private RobotHardware robotHardware;

    public final double DRIVE_SPEED_MAX = 1.0;
    public double driveSpeedControl = DRIVE_SPEED_MAX;

    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;

    @Override
    public void init(HardwareMap hardwareMap) {
        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        fLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fRight.setDirection(DcMotorEx.Direction.FORWARD);
        bLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bRight.setDirection(DcMotorEx.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopAllMotors();
    }
    @Override
    public void stop() { stopAllMotors(); }

    public void stopAllMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    public void setSafePower(DcMotor motor, double targetPower){
        final double SLEW_RATE = 0.2;  // max change per cycle

        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;

        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));

        motor.setPower(currentPower + limitedChange);
    }

    public void setDrivePower(double frLeft, double frRight, double baLeft, double baRight) {
        setSafePower(fLeft,  frLeft  * driveSpeedControl);
        setSafePower(fRight, frRight * driveSpeedControl);
        setSafePower(bLeft,  baLeft  * driveSpeedControl);
        setSafePower(bRight, baRight * driveSpeedControl);
    }

    public void driveRobot(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 1.1;

        // deadzone
        y  = Math.abs(y)  > 0.10 ? y  : 0;
        x  = Math.abs(x)  > 0.10 ? x  : 0;
        rx = Math.abs(rx) > 0.10 ? rx : 0;

        // stop if no input
        if (y == 0 && x == 0 && rx == 0) {
            stopAllMotors();
            return;
        }

        double frLeft  = y + x + rx;
        double frRight = y - x - rx;
        double baLeft  = y - x + rx;
        double baRight = y + x - rx;

        setDrivePower(frLeft, frRight, baLeft, baRight);
    }
    public void moveForward(double power) {
        setDrivePower(power, power, power, power);
    }
    public void driveForwardTimed(double power, long durationMs) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMs) {
            moveForward(power);
            try { Thread.sleep(10); } catch (InterruptedException e) {}
        }
        stopAllMotors();
    }
}