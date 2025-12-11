package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;


@Configurable
public class Intake extends SubsystemBase {
    private final MotorEx intake;
    private final MotorEx transport;
    public static double maxSpeed = 1.0; //Motor.GoBILDA.RPM_1150.getAchievableMaxTicksPerSecond();

    private ColorRangeSensor transportSensor = null;
    private ColorRangeSensor intakeSensor = null;

    private final Telemetry telemetry;
    private Timing.Timer intakeTime = null;
    private Timing.Timer reverseTime = null;

    private boolean enableIntake = false;
    private boolean reverseIntake = false;
    private boolean enableTransport = false;


    public Intake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;

        intake = new MotorEx(hardwareMap, "flywheel_intake", Motor.GoBILDA.RPM_1150);
        transport = new MotorEx(hardwareMap, "flywheel_intake2", Motor.GoBILDA.RPM_312);
        intake.setBuffer(1.0);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setRunMode(Motor.RunMode.RawPower);
        transport.setBuffer(1.0);
        transport.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transport.setRunMode(Motor.RunMode.RawPower);

        transportSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Transport");
        //intakeSensor = hardwareMap.get(ColorRangeSensor.class, "Sensor_Intake");

    }

    public boolean isTransportFull() {
        if (transportSensor == null) return false;
        return Outake.getArtifactColor(transportSensor,20.0) != Outake.ArtifactColor.NOTHING;
    }
    public boolean isIntakeFull() {
        if (intakeSensor == null) return false;
        return Outake.getArtifactColor(intakeSensor,20.0) != Outake.ArtifactColor.NOTHING;
    }

    public void setReverseIntake(boolean set) {
        reverseIntake = set;
    }

    public void startIntakeWheel()
    {
        enableIntake = true;
        intakeTime = new Timing.Timer(100, TimeUnit.MILLISECONDS);
    }
    public void startTransport() {
        enableTransport = true;
    }

    public void StartIntake() {
        startIntakeWheel();
        startTransport();
    }

    public void stopIntakeWheel() {
        enableIntake = false;
        intakeTime= null;
    }
    public void stopTransport() {
        enableTransport = false;
    }
    public void StopIntake() {
        stopIntakeWheel();
        stopTransport();
    }

    public void emptyIntake() {
        if (!reverseIntake) {
            reverseTime = new Timing.Timer(500, TimeUnit.MILLISECONDS);
            setReverseIntake(true);
        }
    }

    @Override
    public void periodic() {

        if (reverseIntake) {
            intake.set(-1.0);
            if (reverseTime.done()) setReverseIntake(false);
        } else if (isStalled()) {
            emptyIntake();
        } else {
            //if intake is enabled set to maxSpeed. Else stop intake
            intake.set(enableIntake ? maxSpeed : 0);
        }
        //if transport is enabled set to maxSpeed. Else stop transport
        transport.set(enableTransport || isTransportFull()? maxSpeed : 0);
    }

    public boolean isStalled() {
        return enableIntake &&
                intakeTime != null && intakeTime.done() &&
                intake.getVelocity()<10;
    }
}