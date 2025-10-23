package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Shooter extends Flywheel {
    private ServoEx rightTrigger;
    private ServoEx leftTrigger;

    public static double la = 0.2;
    public static double ra = 0.2;


    public static double stoppedPos = 0.4;
    public static double triggeredPos = 0.6;
    public static long triggerDelay = 100;

    private double velocity;


    public Shooter(final HardwareMap hMap, TelemetryManager m) {
        super(hMap, m,"flywheel", Motor.GoBILDA.BARE);

        rightTrigger = new ServoEx(hMap, "rt");
        rightTrigger.setInverted(true);
        setRightPosition(stoppedPos);

        leftTrigger = new ServoEx(hMap, "lt");
        setLeftPosition(stoppedPos);

    }


    @Override
    public void periodic() {
        super.periodic();
    }

    public void triggerRight(){
        setRightPosition(triggeredPos);
    }
    public void triggerLeft(){
        setLeftPosition(triggeredPos);
    }

    public void setLeftPosition(double pos) {
        telemetryM.addData("setting Left trigger to", pos);
        leftTrigger.set(pos);
    }
    public void setRightPosition(double pos) {
        telemetryM.addData("setting Right trigger to", pos);
        rightTrigger.set(pos);
    }
    public double getLeftPosition() { return leftTrigger.get();}
    public double getRightPosition() { return rightTrigger.get();}


    public CommandBase buildTriggerCommand(boolean left) {
        if (left) {
            return  new SequentialCommandGroup(
                    new InstantCommand(() -> setLeftPosition(triggeredPos)),
                    new WaitCommand(triggerDelay),
                    new InstantCommand(() -> setLeftPosition(stoppedPos))
            );
        } else {
            return  new SequentialCommandGroup(
                    new InstantCommand(() -> setRightPosition(triggeredPos)),
                    new WaitCommand(triggerDelay),
                    new InstantCommand(() -> setRightPosition(stoppedPos))
            );
        }
    }
}

