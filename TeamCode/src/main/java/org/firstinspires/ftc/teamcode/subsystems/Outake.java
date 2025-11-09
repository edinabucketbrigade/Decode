package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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

    private ColorSensor leftSensor;
    private ColorSensor rightSensor;

    private final MotorEx flywheel = new MotorEx("flywheel_intake").reversed().floatMode();
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01,0,0)
            .basicFF(0.01,0,0.03)
            .build();

    public static double percentage = 1.0;
    public static final double maxVel = 28 * 6000/60; //cpr * rpm / 60 sec/min

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }

    public Outake() {
        leftSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "Left Sensor");
        rightSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "Right Sensor");
        triggerL.getServo().setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void initialize() {
        triggerL.setPosition(resetPosition);
        triggerR.setPosition(resetPosition);
        controller.setGoal(new KineticState(Double.POSITIVE_INFINITY, 0));
    }

    @Override
    public void periodic() {
        flywheel.setPower(controller.calculate(flywheel.getState()));
        ActiveOpMode.telemetry().addData("Left Sensor", "%d-%d-%d",
                leftSensor.red(), leftSensor.blue(), leftSensor.green());
        ActiveOpMode.telemetry().addData("Right Sensor", "%d-%d-%d",
                rightSensor.red(), rightSensor.blue(), rightSensor.green());
        ActiveOpMode.telemetry().addData("Outake velocity", flywheel.getVelocity());
    }

    public final Command off = new RunToVelocity(controller, 0.0)
            .requires(this).named("OutakeOff");
    public final Command on = new RunToVelocity(controller, maxVel*percentage)
            .requires(this).named("OutakeOn");
    public final Command updateVel = new InstantCommand(() ->
            controller.setGoal(new KineticState(Double.POSITIVE_INFINITY,
                    145.6*1150/60*percentage))
    ).requires(this).named("UpdateOutake");

    public final Command waitUntilFast =
            new WaitUntil(() -> (flywheel.getVelocity() / maxVel) > 0.95);

    public final Command shootL = new SequentialGroup(
            waitUntilFast,
            new SetPosition(triggerL, triggerPosition).requires(this),
            new Delay(triggerDelay),
            new SetPosition(triggerL, resetPosition).requires(this)
    );
    public final Command shootR = new SequentialGroup(
            waitUntilFast,
            new SetPosition(triggerR, triggerPosition).requires(this),
            new Delay(triggerDelay),
            new SetPosition(triggerR, resetPosition).requires(this)
    );

    private ArtifactColor getLeftColor() {
        if (leftSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (leftSensor.red() > 150 && leftSensor.blue() > 150)
            return ArtifactColor.PURPLE;
        return ArtifactColor.NOTHING;
    }

    private ArtifactColor getRightColor() {
        if (rightSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (rightSensor.red() > 150 && rightSensor.blue() > 150)
            return ArtifactColor.PURPLE;
        return ArtifactColor.NOTHING;
    }

    public Command shootLoaded = new IfElseCommand(
            () -> getLeftColor() != ArtifactColor.NOTHING,
            shootL,
            new IfElseCommand(
                    () -> getRightColor() != ArtifactColor.NOTHING,
                    shootR
            ));


    public Command shootPurple = new IfElseCommand(
            () -> getLeftColor() == ArtifactColor.PURPLE,
            shootL,
            shootR
    );

    public Command shootGreen = new IfElseCommand(
            () -> getLeftColor() == ArtifactColor.GREEN,
            shootL,
            shootR
    );

}

