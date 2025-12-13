package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Outake extends SubsystemBase {
    private MotorEx flywheel;

    private DcMotorEx fly;
    private PIDFController pidf;
    private final VoltageSensor voltageSensor;

    private final ServoEx triggerL;
    private final ServoEx triggerR;
    private final ColorRangeSensor leftSensor;
    private final ColorRangeSensor rightSensor;
    public static double maxSpeed = 2140;


    public static double speed = 1.0;

    public static double resetPosition = 0.4;
    public static double triggerPosition = .9;
    public static long triggerDelay = 100;
    private double setSpeed = 0;

    private boolean enableFlywheel = false;

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }

    public static double distanceToBall = 13.0;

    private final Telemetry telemetry;

    public Outake(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;

        fly = hardwareMap.get(DcMotorEx.class, "flywheel_outake");
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        pidf = new PIDFController(0.001, 0.0, 0.0001, 0.00048);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        triggerL = new ServoEx(hardwareMap, "Servo_Left", 0, 1);
        triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
        triggerR.setInverted(true);
        triggerL.set(resetPosition);
        triggerR.set(resetPosition);

        leftSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Left");
        rightSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Right");

    }


    @Override
    public void periodic() {
        telemetry.addData("Loaded", "%-7s - %-7s", getLeftColor().name(), getRightColor().name());

        if (fly != null) {
            setSpeed = speed * maxSpeed;
            if (enableFlywheel)
                fly.setPower(
                        pidf.calculate(fly.getVelocity(), setSpeed) * 12 / voltageSensor.getVoltage()
                );
            else
                fly.setPower(0);

//            telemetry.addData("Outake velocity", "%f/%f (%f%%)",
//                    fly.getVelocity(), setSpeed,
//                    (fly.getVelocity() / setSpeed * 100)
//            );

        }

//        telemetry.addData("Speed", speed);
    }

    public void StartOutake() {
        enableFlywheel = true;
    }

    public void StopOutake() {
        enableFlywheel = false;
    }

    public static ArtifactColor getArtifactColor(ColorRangeSensor sensor, double maxDistance) {
        if (sensor.getDistance(DistanceUnit.CM) > maxDistance ||
                sensor.getDistance(DistanceUnit.CM) == DistanceSensor.distanceOutOfRange ||
                Double.valueOf(sensor.getDistance(DistanceUnit.CM)).isNaN()
        ) return ArtifactColor.NOTHING;
        if (sensor.green() > sensor.blue())
            return ArtifactColor.GREEN;
        return ArtifactColor.PURPLE;
    }

    private ArtifactColor getLeftColor() {
        return getArtifactColor(leftSensor, distanceToBall);
    }

    private ArtifactColor getRightColor() {
        return getArtifactColor(rightSensor, distanceToBall);
    }

    public boolean isLoaded() {
        return getLeftColor() != ArtifactColor.NOTHING || getRightColor() != ArtifactColor.NOTHING;
    }
    public boolean isFull() {
        return getLeftColor() != ArtifactColor.NOTHING && getRightColor() != ArtifactColor.NOTHING;
    }

    public void SettriggerL(double position) {
        triggerL.set(position);
    }

    public void SettriggerR(double position) {
        triggerR.set(position);
    }

    public final Command waitUntilFast() {
        return new ParallelRaceGroup(
                new WaitUntilCommand(() ->
                        (fly.getVelocity() / setSpeed) > 0.90),
                new WaitCommand(1500)
        );
    }

    public CommandBase shootL() {
        return new SequentialCommandGroup(
                waitUntilFast(),
                new InstantCommand(() -> SettriggerL(triggerPosition)),
                new WaitCommand(triggerDelay),
                new InstantCommand(() -> SettriggerL(resetPosition))
        );
    }

    public CommandBase shootR() {
        return new SequentialCommandGroup(
                waitUntilFast(),
                new InstantCommand(() -> SettriggerR(triggerPosition)),
                new WaitCommand(triggerDelay),
                new InstantCommand(() -> SettriggerR(resetPosition))
        );
    }

    public CommandBase shootBoth() {
        return new ParallelCommandGroup(
                waitUntilFast(),
                new InstantCommand(() -> SettriggerL(triggerPosition)),
                new InstantCommand(() -> SettriggerR(triggerPosition)),
                new WaitCommand(triggerDelay),
                new InstantCommand(() -> SettriggerL(resetPosition)),
                new InstantCommand(() -> SettriggerR(resetPosition))
        );
    }

    public CommandBase shootLoaded() {
        return new ConditionalCommand(
                shootL(),
                new ConditionalCommand(
                        shootR(),
                        new WaitCommand(1),
                        () -> getRightColor() != ArtifactColor.NOTHING
                ),
                () -> getLeftColor() != ArtifactColor.NOTHING
        );
    }

    public CommandBase shootPurple() {
        return new ConditionalCommand(
                shootL(),
                shootR(),
                () -> getLeftColor() == ArtifactColor.PURPLE
        );

    }

    public CommandBase shootGreen() {
        return new ConditionalCommand(
                shootL(),
                shootR(),
                () -> getLeftColor() == ArtifactColor.GREEN
        );
    }

}

