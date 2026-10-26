package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Mecnum;
import org.firstinspires.ftc.teamcode.subSystems.Outtake;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;

@TeleOp(name = "TeleOp Red", group = "comp")
public class TeleOPRed extends OpMode {
    private static final double TRIGGER_THRESHOLD = 0.1;

    private final Mecnum drive = new Mecnum();
    private final Intake intake = new Intake();
    private final Transfer transfer = new Transfer();
    private final Outtake shooter = new Outtake();

    private boolean rightTriggerWasPressed;

    @Override
    public void init() {
        drive.init(hardwareMap);
        drive.driveSpeedControl = 1;
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        shooter.init(hardwareMap);

        rightTriggerWasPressed = false;

        telemetry.addLine("Drivetrain, intake, transfer and flywheel ready");
        telemetry.addLine("Controller 2: A forward / B reverse");
        telemetry.addLine("Controller 2: RT flywheel ON / LT OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.driveRobot(gamepad1);

        if (gamepad2.b) {
            intake.intakeReverse();
            transfer.intakeReverse();
        } else if (gamepad2.a) {
            intake.intakeForwards();
            transfer.intakeForwards();
        } else {
            intake.stop();
            transfer.stop();
        }

        boolean rightTriggerPressed =
                gamepad2.right_trigger > TRIGGER_THRESHOLD;

        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            shooter.disableFlywheel();
        } else if (rightTriggerPressed && !rightTriggerWasPressed) {
            shooter.enableFlywheel();
        }

        rightTriggerWasPressed = rightTriggerPressed;

        telemetry.addData(
                "Flywheel",
                shooter.isFlywheelOn() ? "ON" : "OFF"
        );
        telemetry.addData("Flywheel power", shooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        drive.stop();
        intake.stop();
        transfer.stop();
    }
}