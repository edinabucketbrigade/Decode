package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;


@Configurable
public class BucketRobot extends Robot {
    private Intake intake;
    private Shooter shooter;
    private TelemetryManager telemetryM;

    public BucketRobot(HardwareMap hMap, TelemetryManager m){
        telemetryM = m;
        shooter = new Shooter(hMap, telemetryM);
        intake = new Intake(hMap, telemetryM);
        register(shooter, intake);
    }
    public void enableIntake() {
         intake.spinUp();
    }
    public void disableIntake() {
         intake.spinDown();
    }
    public void enableShooter() {
         shooter.spinUp();
    }
    public void disableShooter() {
         shooter.spinDown();
    }
    public void shootRight() {
         shooter.triggerRight();
    }
    public void shootLeft() {
         shooter.triggerLeft();
    }
    public void setLeft() {
        shooter.setLeft();
    }
    public void setRight() {
        shooter.setRight();
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
        return new InstantCommand(() -> shooter.triggerRight());
    }
    public Command shootLeftCommand() {
        return new InstantCommand(() -> shooter.triggerLeft());
    }

}
