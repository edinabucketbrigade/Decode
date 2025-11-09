package org.firstinspires.ftc.teamcode.subsystems;

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
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Outake extends SubsystemBase {
    private Telemetry telemetryM;
    private MotorEx flywheel;
    private ServoEx triggerL;
    private ServoEx triggerR;
    private SensorColor leftSensor;
    private SensorColor rightSensor;
    private double maxSpeed;

    public static double kP = 20;
    public static double kV = 0.7;
    public static double speed = 1.0;

    public static double resetPosition = 0.4;
    public static double triggerPosition = 1.0;
    public static long triggerDelay = 150;
    public boolean isRunning;

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NOTHING
    }


    public Outake(HardwareMap hardwareMap, Telemetry m) {
        telemetryM = m;

        flywheel = new MotorEx(hardwareMap, "flywheel_outake", Motor.GoBILDA.BARE);
        flywheel.setBuffer(1.0);
        maxSpeed = flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel.setInverted(true);
        flywheel.setVeloCoefficients(kP, 0, 0);
        flywheel.setFeedforwardCoefficients(0, kV);
        isRunning = false;

        triggerL = new ServoEx(hardwareMap, "Servo_Left", 0, 1);
        triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
        triggerL.setInverted(true);
        triggerL.set(resetPosition);
        triggerR.set(resetPosition);

        leftSensor = new SensorColor(hardwareMap, "Sensor_Left");
        rightSensor = new SensorColor(hardwareMap, "Sensor_Right");
    }

    private ArtifactColor getLeftColor() {
        if (leftSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (leftSensor.red() > 150 && leftSensor.blue() > 150)
            return ArtifactColor.PURPLE;

        return ArtifactColor.NOTHING;
    }

    @Override
    public void periodic() {
        telemetryM.addData("Left Sensor", "%d-%d-%d",
                leftSensor.red(), leftSensor.blue(), leftSensor.green());
        telemetryM.addData("Right Sensor", "%d-%d-%d",
                rightSensor.red(), rightSensor.blue(), rightSensor.green());

        telemetryM.addData("Outake velocity", "%f - $f", flywheel.getVelocity(),
                (flywheel.getVelocity() / (speed * maxSpeed)));
    }

    private ArtifactColor getRightColor() {
        if (rightSensor.green() > 150)
            return ArtifactColor.GREEN;
        if (rightSensor.red() > 150 && rightSensor.blue() > 150)
            return ArtifactColor.PURPLE;

        return ArtifactColor.NOTHING;
    }

    public void StartOutake() {
        flywheel.setVelocity(speed * maxSpeed);
    }


    public void StopOutake() {
        flywheel.set(0);
    }


    public void SettriggerL(double position) {
        triggerL.set(position);
    }

    public void SettriggerR(double position) {
        triggerR.set(position);
    }

    public final Command waitUntilFast = new WaitUntilCommand(() ->
             (flywheel.getVelocity() / (speed * maxSpeed)) > 0.95
            );

    public CommandBase shootL() {
        return new SequentialCommandGroup(
                waitUntilFast,
                new InstantCommand(() -> SettriggerL(triggerPosition)),
                new WaitCommand(triggerDelay),
                new InstantCommand(() -> SettriggerL(resetPosition))
        );

    }

    public CommandBase shootR() {
        return new SequentialCommandGroup(
                waitUntilFast,
                new InstantCommand(() -> SettriggerR(triggerPosition)),
                new WaitCommand(triggerDelay),
                new InstantCommand(() -> SettriggerR(resetPosition))
        );
    }

    public CommandBase shootLoaded() {
        if (getLeftColor() != ArtifactColor.NOTHING)
            return shootL();
        if (getRightColor() != ArtifactColor.NOTHING)
            return shootR();
        return new WaitCommand(1);
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
                shootR(),
                shootL(),
                () -> getRightColor() == ArtifactColor.GREEN
        );
    }

    public void ToggleOutake() {
        if (!isRunning) {
            isRunning = true;
            StartOutake();
        } else {
            isRunning = false;
            StopOutake();

        }
    }
}

