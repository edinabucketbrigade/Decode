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
    public static double maxSpeed = Motor.GoBILDA.RPM_1150.getAchievableMaxTicksPerSecond();
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kV = 1.0;
    public static double kD = 0.0;

    public static double speed = Motor.GoBILDA.RPM_1150.getAchievableMaxTicksPerSecond();

    private Telemetry telemetry;
    private double setSpeed = 0;

    public Intake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;

        flywheel = new MotorEx(hardwareMap, "flywheel_intake", Motor.GoBILDA.RPM_1150);
        flywheel.setBuffer(1.0);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(kP, kI, kD);
        flywheel.setFeedforwardCoefficients(0, kV, 0);
    }

    public void StartIntake() {
        setSpeed = maxSpeed;
    }

    public void StopIntake() {
        setSpeed = 0;
    }


    @Override
    public void periodic() {
        if (flywheel != null) {
            //if the flywheel is stuck reverse for a bit and try again
            if (setSpeed > 0 && flywheel.getVelocity() < 1)
                flywheel.setVelocity(-10.0);
            else if (setSpeed < 0 && flywheel.get() < -5.0)
                flywheel.setVelocity(-10.0);
            else
                flywheel.setVelocity(setSpeed);
        }
    }
}