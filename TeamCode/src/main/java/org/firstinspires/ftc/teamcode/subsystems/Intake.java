package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Configurable
public class Intake extends SubsystemBase {
    private MotorEx flywheel;
    private double maxSpeed;
    public static double kP = 0.001;
    public static double kS = 0.0;
    public static double kV = 1.0;
    public static double kA = 0.0;

    public static double speed = 1.0;

    public boolean isRunning;
    private Telemetry telemetry;
    private double setSpeed = 0;

    public Intake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;

        flywheel = new MotorEx(hardwareMap, "flywheel_intake", Motor.GoBILDA.RPM_1150);
        flywheel.setBuffer(1.0);
        maxSpeed = flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(kP, 0, 0);
        flywheel.setFeedforwardCoefficients(kS, kV, kA);
        isRunning = false;
    }

    public void StartIntake() {
        setSpeed = speed * maxSpeed;
        flywheel.setVelocity(setSpeed);

    }

    public void StopIntake() {
        setSpeed = 0; flywheel.set(setSpeed);
    }

    public void ToggleIntake() {
        if (!isRunning) {
            isRunning = true;
            StartIntake();
        } else {
            isRunning = false;
            StopIntake();
        }
    }


    @Override
    public void periodic() {
        if (flywheel != null)
            telemetry.addData("Intake velocity", "%f (%f) -> %f",
                    flywheel.getVelocity(),
                    (flywheel.getVelocity() / (speed * maxSpeed)),
                    setSpeed);
    }
}