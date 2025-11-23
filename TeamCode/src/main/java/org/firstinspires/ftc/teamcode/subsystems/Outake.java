package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Outake extends SubsystemBase {
    private MotorEx flywheel;

    private DcMotorEx fly;
    private PIDFController pidf;
    private VoltageSensor voltageSensor;

    private ServoEx triggerL;
    private ServoEx triggerR;
    private ColorRangeSensor leftSensor;
    private ColorRangeSensor rightSensor;
    public static double maxSpeed = 2140;

    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kV = 1.0;
    public static double kD = 0.0001;
    public static double speed = 1.0;

    public static boolean waitToShoot = true;

    public static double resetPosition = 0.4;
    public static double triggerPosition = 1.0;
    public static long triggerDelay = 150;
    private double setSpeed = 0;

    private boolean enableFlywheel = false;

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }

    public static double distanceToBall = 13.0;

    private Telemetry telemetry;

    public static boolean useOldFlywheel = false;

    public Outake(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;

        if (useOldFlywheel) {
            flywheel = new MotorEx(hardwareMap, "flywheel_outake", Motor.GoBILDA.BARE);
            flywheel.setBuffer(1.0);
            flywheel.setRunMode(Motor.RunMode.VelocityControl);
            flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            flywheel.setInverted(true);
            flywheel.setVeloCoefficients(kP, kI, kD);
            flywheel.setFeedforwardCoefficients(0, kV, 0);

        } else {
            fly = hardwareMap.get(DcMotorEx.class, "flywheel_outake");
            fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fly.setDirection(DcMotorSimple.Direction.REVERSE);
            pidf = new PIDFController(0.001, 0.0, 0.0001, 0.00048);
        }
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

        if (useOldFlywheel) {
            if (flywheel != null) {
                setSpeed = speed * maxSpeed;
                if (enableFlywheel)
                    flywheel.setVelocity(setSpeed);
                else
                    flywheel.setVelocity(0);

                telemetry.addData("Outake velocity", "%f/%f (%f%%)",
                        flywheel.getVelocity(),setSpeed,
                        (flywheel.getVelocity() / setSpeed * 100)
                        );

            }

        } else {
            if (fly != null) {
                setSpeed = speed * maxSpeed;
                if (enableFlywheel)
                    fly.setPower(
                            pidf.calculate(fly.getVelocity(), setSpeed) * 12 / voltageSensor.getVoltage()
                    );
                else
                    fly.setPower(0);

                telemetry.addData("Outake velocity", "%f/%f (%f%%)",
                        fly.getVelocity(),setSpeed,
                        (fly.getVelocity() / setSpeed * 100)
                        );

            }
        }
        telemetry.addData("Speed", speed);
    }

    public void StartOutake() {
        enableFlywheel = true;
    }

    public void StopOutake() {
        enableFlywheel = false;
    }

    private ArtifactColor getLeftColor() {
        if (leftSensor.getDistance(DistanceUnit.CM) > distanceToBall ||
                leftSensor.getDistance(DistanceUnit.CM) == DistanceSensor.distanceOutOfRange ||
                new Double(leftSensor.getDistance(DistanceUnit.CM)).isNaN()
        )
            return ArtifactColor.NOTHING;
        if (leftSensor.green() > leftSensor.blue())
            return ArtifactColor.GREEN;
        return ArtifactColor.PURPLE;
    }

    private ArtifactColor getRightColor() {
        if (rightSensor.getDistance(DistanceUnit.CM) > distanceToBall ||
                rightSensor.getDistance(DistanceUnit.CM) == DistanceSensor.distanceOutOfRange ||
                new Double(rightSensor.getDistance(DistanceUnit.CM)).isNaN())
            return ArtifactColor.NOTHING;
        if (rightSensor.green() > rightSensor.blue())
            return ArtifactColor.GREEN;
        return ArtifactColor.PURPLE;
    }

    public boolean isLoaded() {
        return getLeftColor() != ArtifactColor.NOTHING || getRightColor() != ArtifactColor.NOTHING;
    }

    public void SettriggerL(double position) {
        triggerL.set(position);
    }

    public void SettriggerR(double position) {
        triggerR.set(position);
    }

    public final Command waitUntilFast() {
        if (waitToShoot)
            return new ParallelRaceGroup(
                    new WaitUntilCommand(() ->
                            (flywheel.getVelocity() / setSpeed) > 0.90),
                    new WaitCommand(1500)
            );
        else return new WaitCommand(1);
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

