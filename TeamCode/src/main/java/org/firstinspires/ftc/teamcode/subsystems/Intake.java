package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


@Configurable
public class Intake extends SubsystemBase {
    private MotorEx flywheel;
    private double maxSpeed;
    public static double kP = 20;
    public static double kV = 0.7;
    public static double speed = 1.0;

    public boolean isRunning;
    public Intake(HardwareMap hardwareMap) {

        flywheel = new MotorEx(hardwareMap, "flywheel_intake", Motor.GoBILDA.RPM_1150);
        flywheel.setBuffer(1.0);
        maxSpeed = flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(kP, kV, 0);
        isRunning = false;
    }

    public void StartIntake() {

        flywheel.setVelocity(speed*maxSpeed);


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