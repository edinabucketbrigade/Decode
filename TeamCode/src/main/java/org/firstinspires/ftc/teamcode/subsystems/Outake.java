package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Outake extends SubsystemBase {
    private MotorEx flywheel;
    private ServoEx triggerL;
    private ServoEx triggerR;
    private ColorRangeSensor leftSensor;
    private ColorRangeSensor rightSensor;
    public static double maxSpeed = 2140;

    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kV = 1.3;
    public static double kD = 0.0;
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

    public static double distanceToBall = 10.0;

    private Telemetry telemetry;

    public Outake(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        flywheel = new MotorEx(hardwareMap, "flywheel_outake", Motor.GoBILDA.BARE);
        flywheel.setBuffer(1.0);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheel.setInverted(true);
        flywheel.setVeloCoefficients(kP, kI, kD);
        flywheel.setFeedforwardCoefficients(0, kV, 0);

        triggerL = new ServoEx(hardwareMap, "Servo_Left", 0, 1);
        triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
        triggerL.setInverted(true);
        triggerL.set(resetPosition);
        triggerR.set(resetPosition);


        leftSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Left");
        rightSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Right");

    }


    @Override
    public void periodic() {
        telemetry.addData("Loaded","%-7s - %-7s", getLeftColor().name(),getRightColor().name());



        if (flywheel != null) {
            setSpeed = speed * maxSpeed;
            if (enableFlywheel)
                flywheel.setVelocity(setSpeed);
            else
                flywheel.setVelocity(0);

            telemetry.addData("Outake velocity", "%f (%f%%) -> %f",
                    flywheel.getVelocity(),
                    (flywheel.getVelocity() / setSpeed * 100),
                    setSpeed);
            telemetry.addData("Speed", speed);

        }
    }

    public void StartOutake() {
        enableFlywheel = true;
    }

    public void StopOutake() {
        enableFlywheel = false;
    }

    private ArtifactColor getLeftColor() {
        if (leftSensor.getDistance(DistanceUnit.CM) > distanceToBall ||
                leftSensor.getDistance(DistanceUnit.CM) == DistanceSensor.distanceOutOfRange)
            return ArtifactColor.NOTHING;
        if (leftSensor.green() > leftSensor.blue())
            return ArtifactColor.GREEN;
        return ArtifactColor.PURPLE;
    }

    private ArtifactColor getRightColor() {
        if (rightSensor.getDistance(DistanceUnit.CM) > distanceToBall ||
                rightSensor.getDistance(DistanceUnit.CM) == DistanceSensor.distanceOutOfRange)
            return ArtifactColor.NOTHING;
        if (rightSensor.green() > rightSensor.blue())
            return ArtifactColor.GREEN;
        return ArtifactColor.PURPLE;
    }


    public void SettriggerL(double position) {
        triggerL.set(position);
    }

    public void SettriggerR(double position) {
        triggerR.set(position);
    }

    public final Command waitUntilFast() {
        if (waitToShoot)
            return new WaitUntilCommand(() ->
                (flywheel.getVelocity() / setSpeed) > 0.80
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
                () -> getRightColor() == ArtifactColor.GREEN
        );
    }

}

