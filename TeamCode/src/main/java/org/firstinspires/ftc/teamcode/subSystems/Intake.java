package org.firstinspires.ftc.teamcode.subSystems;

// com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Configurable
public class Intake implements Subsystems {

    private DcMotor intakeMotor;

    public static double INTAKE_POWER = 0.9;
    public static double OUTTAKE_POWER = -0.9;

    @Override
    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    @Override
    public void stop(){
        intakeMotor.setPower(0);
    }

    public void intakeForwards(){
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void intakeReverse(){
        intakeMotor.setPower(OUTTAKE_POWER);
    }
}