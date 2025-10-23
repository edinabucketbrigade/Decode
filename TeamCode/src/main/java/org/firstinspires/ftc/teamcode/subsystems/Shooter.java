package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Shooter extends SubsystemBase {
    private MotorEx flywheel;
    private ServoEx rightTrigger;
    private ServoEx leftTrigger;
    private TelemetryManager telemetryM;

    public static double stoppedVelocity = 0;
    public static double runningVelocity = 6000.0;
    public static double kP = 20;
    public static double kV = 0.7;

    public static double triggerAngle = 30.0;
    public static double minRAngle = 0.0;
    public static double maxRAngle = minRAngle+triggerAngle;
    public static double minLAngle = 180.0;
    public static double maxLAngle = minLAngle+triggerAngle;

    public static double la = 0.0;
    public static double ra = 0.0;


    private double velocity;
    private boolean rightTriggerState, leftTriggerState;

    public Shooter(final HardwareMap hMap, TelemetryManager m) {
        flywheel = new MotorEx(hMap, "flywheel");
        rightTrigger = new ServoEx(hMap, "rt", minRAngle, maxRAngle);
        leftTrigger = new ServoEx(hMap, "lt", minLAngle, maxLAngle);
        telemetryM = m;

        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(kP, 0, 0);
        flywheel.setFeedforwardCoefficients(0, kV);

    }

    public void setSpeed(double vel){
        telemetryM.debug("setting shooter velocity", vel);
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
        telemetryM.debug("setting shooter motor velocity", velocity);
        if (rightTriggerState) {
            rightTriggerState = false;
            rightTrigger.set(0);
        }
        if (leftTriggerState) {
            leftTriggerState = false;
            leftTrigger.set(0);
        }
        flywheel.setVelocity(velocity);
    }

    public void triggerRight(){
        telemetryM.debug("adding Trigger Right");

        rightTriggerState = true;
        rightTrigger.set(1);
    }

    public void triggerLeft(){
        telemetryM.debug("adding Trigger Left");
        leftTriggerState = true;
        leftTrigger.set(1);
    }

    public void setLeft()
    {
        leftTrigger.set(la);
    }
    public void setRight()
    {
        rightTrigger.set(ra);
    }
}
