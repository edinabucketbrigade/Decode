package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private final MotorEx flywheel = new MotorEx("flywheel_intake").floatMode();
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01,0,0)
            .basicFF(0.01,0,0.03)
            .build();

    public static double percentage = 1.0;
    public static final double maxVel = 145.6*1150/60; //cpr * rpm / 60 sec/min

    public final Command off = new RunToVelocity(controller, 0.0)
            .requires(this).named("IntakeOff");

    public final Command on = new RunToVelocity(controller, maxVel*percentage)
            .requires(this).named("IntakeOn");

    public final Command updateVel = new InstantCommand(() ->
        controller.setGoal(new KineticState(Double.POSITIVE_INFINITY,
                145.6*1150/60*percentage))
    ).requires(this).named("UpdateIntake");

    @Override
    public void initialize() {
        controller.setGoal(new KineticState(Double.POSITIVE_INFINITY, 0));
    }
    @Override
    public void periodic() {
        flywheel.setPower(controller.calculate(flywheel.getState()));
        ActiveOpMode.telemetry().addData("Intake velocity", flywheel.getVelocity());
    }
}