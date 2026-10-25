package org.firstinspires.ftc.teamcode.subSystems;

//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Configurable
public class robotHardware {

    public LinearOpMode myOpMode;

    // Subsystems
    public Mecnum mecnum;
    //public Intake intake;
    //public Outtake outtake;
    //public Turret turret;

    public Subsystems[] subsystems;

    public robotHardware(LinearOpMode opMode) {
        myOpMode = opMode;

        // Initialize subsystems
        mecnum = new Mecnum();
        //intake = new Intake();
        //outtake = new Outtake();
        //turret = new Turret();

        subsystems = new Subsystems[]{mecnum}; //intake, outtake, turret
    }

    public void init() {
        for (Subsystems subsystem : subsystems) {
            subsystem.init(myOpMode.hardwareMap);
        }

        myOpMode.telemetry.addData("Status", "All subsystems initialized");
        myOpMode.telemetry.update();
    }

    public void stop() {
        for (Subsystems subsystem : subsystems) {
            subsystem.stop();
        }

        myOpMode.telemetry.addData("Status", "All subsystems stopped");
        myOpMode.telemetry.update();
    }
}