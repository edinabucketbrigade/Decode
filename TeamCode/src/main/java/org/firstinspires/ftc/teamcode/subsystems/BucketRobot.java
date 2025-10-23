package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.hardware.SimpleServo;


@Configurable
public class BucketRobot extends Robot {
    private Flywheel intake;
    private Shooter shooter;
    private TelemetryManager telemetryM;

    SimpleServo s;

    public BucketRobot(HardwareMap hMap, TelemetryManager m){
        telemetryM = m;
        shooter = new Shooter(hMap, telemetryM);
        intake = new Flywheel(hMap, telemetryM, "intake");
        register(shooter, intake);
    }
    public Command enableIntakeCommand() {

        return new InstantCommand(() -> intake.spinUp());
    }
    public Command disableIntakeCommand() {
        return new InstantCommand(() -> intake.spinDown());
    }
    public Command enableShooterCommand() {
        return new InstantCommand(() -> shooter.spinUp());
    }
    public Command disableShooterCommand() {
        return new InstantCommand(() -> shooter.spinDown());
    }

    public Command shootRightCommand() {
        return shooter.buildTriggerCommand(false);
    }
    public Command shootLeftCommand() {
        return shooter.buildTriggerCommand(true);
    }

}
