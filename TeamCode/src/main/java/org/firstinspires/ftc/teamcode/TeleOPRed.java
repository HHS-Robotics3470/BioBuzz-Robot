package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Mecnum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;

@TeleOp(name = "TeleOp Red", group = "comp")
public class TeleOPRed extends OpMode {

    private final Mecnum drive = new Mecnum();
    private final Intake intake = new Intake();
    private final Transfer transfer = new Transfer();

    @Override
    public void init() {
        drive.init(hardwareMap);
        drive.driveSpeedControl = 1;
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        telemetry.addLine("Drivetrain and intake ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.driveRobot(gamepad1);

        if (gamepad2.x) {
            intake.intakeReverse();
            transfer.intakeReverse();
        } else {
            intake.stop();
            transfer.stop();
        }

        if (gamepad2.a) {
            intake.intakeForwards();
            transfer.intakeForwards();
        } else {
            intake.stop();
            transfer.stop();
        }
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        transfer.stop();
    }
}