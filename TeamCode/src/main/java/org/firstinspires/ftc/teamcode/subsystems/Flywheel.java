package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Flywheel extends SubsystemBase {
    public static double stoppedVelocity = 0;
    public static double runningVelocity = 6000.0;
    public static double kP = 0.05;
    public static double kI = 0.01;
    public static double kD = 0.31;

    private double velocity;
    private MotorEx intakeWheels;
    protected TelemetryManager telemetryM;


    public Flywheel(final HardwareMap hMap, TelemetryManager m, String motorName) {
        this(hMap,m, motorName, Motor.GoBILDA.RPM_435);
    }
    public Flywheel(final HardwareMap hMap, TelemetryManager m, String motorName, Motor.GoBILDA motor) {
        telemetryM = m;

        intakeWheels = new MotorEx(hMap, motorName, motor);
        intakeWheels.setRunMode(Motor.RunMode.VelocityControl);
        intakeWheels.setVeloCoefficients(kP, kI, kD);

        velocity = stoppedVelocity;
    }


    public void setSpeed(double vel){
        telemetryM.debug("setting intake velocity", vel);
        velocity = vel;
    }
    public void spinUp() {
         setSpeed(runningVelocity);
    }
    public void spinDown() {
         setSpeed(stoppedVelocity);
    }


    @Override
    public void periodic() {
        telemetryM.debug("setting intake motor velocity", velocity);
        intakeWheels.setVelocity(velocity);
    }
}
