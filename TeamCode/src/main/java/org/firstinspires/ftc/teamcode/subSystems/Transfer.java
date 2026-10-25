package org.firstinspires.ftc.teamcode.subSystems;

// com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Configurable
public class Transfer implements Subsystems {

    private DcMotor transferMotor;

    public static double TRANSFER_POWER = 1;
    public static double OUTTAKE_POWER = -1;

    @Override
    public void init(HardwareMap hardwareMap){
        transferMotor = hardwareMap.get(DcMotor.class,"transferMotor");
        transferMotor.setDirection(DcMotor.Direction.FORWARD );
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setPower(0);
    }


    public void intakeForwards(){
        transferMotor.setPower(TRANSFER_POWER);
    }
    public void stop(){
        transferMotor.setPower(0);
    }
    public void intakeReverse(){
        transferMotor.setPower(OUTTAKE_POWER);
    }
}