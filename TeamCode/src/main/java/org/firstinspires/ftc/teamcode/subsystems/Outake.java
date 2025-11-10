package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;


import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@Configurable
public class Outake implements Subsystem {
    public static final Outake INSTANCE = new Outake();

    private final ServoEx triggerL = new ServoEx("Servo_Left");
    private final ServoEx triggerR = new ServoEx("Servo_Right");
    public static double triggerDelay = 0.150;
    public static double resetPosition = 0.4;
    public static double triggerPosition = 1.0;

    private ColorSensor leftSensor, rightSensor;

    private final MotorEx flywheel = new MotorEx("flywheel_intake").reversed().floatMode();

    public static double kP = 0.01, kI = 0.0, kD = 0.1;
    public static double kV = 0.01, kA = 0.0, kS = 0.01;
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();

    public static double percentage = 1.0;
    public static final double maxVel = 28 * 6000 / 60; //cpr * rpm / 60 sec/min


    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }


    private final boolean sensorsDisabled = true;

    public Outake() {
        if (!sensorsDisabled) {
            leftSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "Left Sensor");
            rightSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "Right Sensor");
        }


    }

    private class IfThenCommand extends Command {
        BooleanSupplier c;
        Command tC, fC;
        Command selected = null;

        public IfThenCommand(BooleanSupplier cond, Command t){
            this(cond,t,new NullCommand());
        }
        public IfThenCommand(BooleanSupplier cond, Command t, Command f) {
            c = cond;
            tC = t;
            fC = f;
        }

        @Override
        public void start() {
            selected = c.getAsBoolean() ? tC : fC;
            selected.start();
        }

        @Override
        public void update() {
            if (selected != null) selected.update();
        }

        @Override
        public boolean isDone() {
            return selected != null && selected.isDone();
        }
    }

    @Override
    public void initialize() {
        triggerL.getServo().setDirection(Servo.Direction.REVERSE);
        triggerL.setPosition(resetPosition);
        triggerR.setPosition(resetPosition);
        controller.setGoal(new KineticState(Double.POSITIVE_INFINITY, 0));
    }

    @Override
    public void periodic() {

        flywheel.setPower(controller.calculate(flywheel.getState()));
        if (!sensorsDisabled) {
            ActiveOpMode.telemetry().addData("Left Sensor", "%d-%d-%d",
                    leftSensor.red(), leftSensor.blue(), leftSensor.green());
            ActiveOpMode.telemetry().addData("Right Sensor", "%d-%d-%d",
                    rightSensor.red(), rightSensor.blue(), rightSensor.green());
        }
        ActiveOpMode.telemetry().addData("Outake velocity", flywheel.getVelocity());

    }

    public final Command off = new RunToVelocity(controller, 0.0)
            .requires(this).named("OutakeOff");
    public final Command on = new RunToVelocity(controller, maxVel * percentage)
            .requires(this).named("OutakeOn");
    public final Command updateVel = new InstantCommand(() ->
            controller.setGoal(new KineticState(Double.POSITIVE_INFINITY,
                    145.6 * 1150 / 60 * percentage))
    ).requires(this).named("UpdateOutake");

    public final Command waitUntilFast() {
        return new WaitUntil(() -> (flywheel.getVelocity() / maxVel) > 0.95);
    }

    public Command shootL = new SequentialGroup(
            waitUntilFast(),
            new SetPosition(triggerL, triggerPosition).requires(this),
            new Delay(triggerDelay),
            new SetPosition(triggerL, resetPosition).requires(this)
    );

    public Command shootR = new SequentialGroup(
            waitUntilFast(),
            new SetPosition(triggerR, triggerPosition).requires(this),
            new Delay(triggerDelay),
            new SetPosition(triggerR, resetPosition).requires(this)
    );


    private ArtifactColor getLeftColor() {
        if (sensorsDisabled) return ArtifactColor.GREEN;
        if (leftSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (leftSensor.red() > 150 && leftSensor.blue() > 150)
            return ArtifactColor.PURPLE;
        return ArtifactColor.NOTHING;
    }

    private ArtifactColor getRightColor() {
        if (sensorsDisabled) return ArtifactColor.GREEN;
        if (rightSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (rightSensor.red() > 150 && rightSensor.blue() > 150)
            return ArtifactColor.PURPLE;
        return ArtifactColor.NOTHING;
    }


    public Command shootLoaded = new IfThenCommand(
            () -> getLeftColor() != ArtifactColor.NOTHING,
            shootL,
            new IfThenCommand(
                    () -> getRightColor() != ArtifactColor.NOTHING,
                    shootR
            ));


    public Command shootPurple = new IfThenCommand(
            () -> getLeftColor() == ArtifactColor.PURPLE,
            shootL,
            shootR
    );

    public Command shootGreen = new IfThenCommand(
            () -> getLeftColor() == ArtifactColor.GREEN,
            shootL,
            shootR
    );


}
