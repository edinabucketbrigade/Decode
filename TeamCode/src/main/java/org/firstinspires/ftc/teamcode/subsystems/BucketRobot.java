package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Configurable
public class BucketRobot extends Robot {
    private Intake intake;
    private Outake outake;
    private Camera camera;
    private Telemetry telemetryM;

    SimpleServo s;

    public BucketRobot(HardwareMap hMap, Telemetry m){
        telemetryM = m;
        outake = new Outake(hMap, telemetryM);
        intake = new Intake(hMap, telemetryM);
        camera = new Camera(hMap, m);
        register(outake, intake, camera);
    }
    public Command enableIntake() {
        return new InstantCommand(() -> intake.StartIntake());
    }
    public Command disableIntake() {
        return new InstantCommand(() -> intake.StopIntake());
    }
    public Command toggleIntake() {
        return new InstantCommand(() -> intake.ToggleIntake());
    }

    public Command enableOutake() {
        return new InstantCommand(() -> outake.StartOutake());
    }
    public Command disableOutake() {
        return new InstantCommand(() -> outake.StopOutake());
    }
    public Command toggleOutake() {
        return new InstantCommand(() -> outake.ToggleOutake());
    }
    public Command shootRight() {
        return outake.shootR();
    }
    public Command shootLeft() {
        return outake.shootL();
    }

    public Command shootGreen() {
        return outake.shootGreen();
    }
    public Command shootPurple() {
        return outake.shootPurple();
    }
    public Command shootLoaded() {
        return outake.shootloaded();
    }

    @Override
    public void run() {
        camera.getPattern();

        super.run();
    }
}
