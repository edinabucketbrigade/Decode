package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;



@Configurable
public class Intake extends SubsystemBase {
    private Motor flywheel;
    public static double kP = 20;
    public static double kV = 0.7;
    public static double speed = 6000.0;


    public boolean isRunning;
    public Intake(HardwareMap hardwareMap) {

        flywheel = new Motor(hardwareMap, "flywheel_Main", Motor.GoBILDA.BARE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(kP, kV, 0);
        isRunning = false;
    }

    public void StartIntake() {

        flywheel.set(speed);


    }

    public void StopIntake() {

        flywheel.set(0);
    }

    public void ToggleIntake(){
       if (!isRunning) {
           isRunning = true;
           StartIntake();
       }else {
           isRunning = false;
           StopIntake();
       }


    }
}