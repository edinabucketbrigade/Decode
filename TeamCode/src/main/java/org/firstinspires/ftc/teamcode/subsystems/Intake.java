package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Intake extends SubsystemBase {
    public static double stoppedVelocity = 0;
    public static double runningVelocity = 6000.0;
    public static double kP = 20;
    public static double kV = 0.7;

    private double velocity;
    private MotorEx intakeWheels;
    private TelemetryManager telemetryM;


    public Intake(final HardwareMap hMap, TelemetryManager m) {
        intakeWheels = new MotorEx(hMap, "intake");
        velocity = stoppedVelocity;
        telemetryM = m;

        intakeWheels.setRunMode(Motor.RunMode.VelocityControl);
        intakeWheels.setVeloCoefficients(kP, 0, 0);
        intakeWheels.setFeedforwardCoefficients(0, kV);

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
