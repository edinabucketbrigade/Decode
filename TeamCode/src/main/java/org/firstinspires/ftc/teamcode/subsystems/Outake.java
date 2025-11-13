package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Outake extends SubsystemBase {
    private MotorEx flywheel;
    private ServoEx triggerL;
    private ServoEx triggerR;
    private SensorColor leftSensor;
    private SensorColor rightSensor;
    private double maxSpeed;

    public static double kP = 0.001;
    public static double kS = 0.0;
    public static double kV = 1.0;
    public static double kA = 0.0;
    public static double speed = 1.0;

    public static double resetPosition = 0.4;
    public static double triggerPosition = 1.0;
    public static long triggerDelay = 150;
    private double setSpeed = 0;

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }


    private final static boolean disableSensors = false;
    private Telemetry telemetry;
    public Outake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;
        flywheel = new MotorEx(hardwareMap, "flywheel_outake", Motor.GoBILDA.BARE);
        flywheel.setBuffer(1.0);
        maxSpeed = flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel.setInverted(true);
        flywheel.setVeloCoefficients(kP, 0, 0);
        flywheel.setFeedforwardCoefficients(kS, kV, kA);

        triggerL = new ServoEx(hardwareMap, "Servo_Left", 0, 1);
        triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
        triggerL.setInverted(true);
        triggerL.set(resetPosition);
        triggerR.set(resetPosition);

        if (!disableSensors) {
            leftSensor = new SensorColor(hardwareMap, "Sensor_Left");
            rightSensor = new SensorColor(hardwareMap, "Sensor_Right");
        }
    }


    @Override
    public void periodic() {

        if (leftSensor != null) telemetry.addData("Left Sensor", "%d-%d-%d",
                leftSensor.red(), leftSensor.blue(), leftSensor.green());

        if (rightSensor != null) telemetry.addData("Right Sensor", "%d-%d-%d",
                rightSensor.red(), rightSensor.blue(), rightSensor.green());

        if (flywheel != null)
            flywheel.set(setSpeed);
            telemetry.addData("Outake velocity", "%f (%f) -> %f",
                    flywheel.getVelocity(),
                    (flywheel.getVelocity() / (speed * maxSpeed)),
                    setSpeed);
    }
    public void StartOutake() {
        setSpeed = speed * maxSpeed;
    }

    public void StopOutake() {
        setSpeed = 0;
    }

    private ArtifactColor getLeftColor() {
        if (disableSensors) return ArtifactColor.GREEN;
        if (leftSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (leftSensor.red() > 150 && leftSensor.blue() > 150)
            return ArtifactColor.PURPLE;

        return ArtifactColor.NOTHING;
    }

    private ArtifactColor getRightColor() {
        if (disableSensors) return ArtifactColor.GREEN;
        if (rightSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (rightSensor.red() > 150 && rightSensor.blue() > 150)
            return ArtifactColor.PURPLE;

        return ArtifactColor.NOTHING;
    }



    public void SettriggerL(double position) {
        triggerL.set(position);
    }

    public void SettriggerR(double position) {
        triggerR.set(position);
    }

    public final Command waitUntilFast() {
        return new WaitUntilCommand(() ->
                (flywheel.getVelocity() / (speed * maxSpeed)) > 0.95
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

