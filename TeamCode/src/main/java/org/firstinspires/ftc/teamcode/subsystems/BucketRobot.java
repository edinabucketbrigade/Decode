package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;


@Configurable
public class BucketRobot extends Robot {
    private Intake intake;
    private Outake outake;
    private Camera camera;


    public enum ARTIFACTPATTERN {
        NONE(0),
        GPP(21),
        PGP(22),
        PPG(23);
        private final int pattern;

        private ARTIFACTPATTERN(int pattern) {
            this.pattern = pattern;
        }

        public int getPattern() {
            return pattern;
        }
    };

    ARTIFACTPATTERN pattern;

    private static boolean disableCamera = false;

    private Telemetry telemetry;
    public BucketRobot(HardwareMap hMap, Telemetry t){
        telemetry = t;
        outake = new Outake(hMap, t);
        intake = new Intake(hMap, t);
        if (!disableCamera) {
            camera = new Camera(hMap, t);
            register(outake, intake, camera);
        } else
            register(outake, intake);

        pattern = ARTIFACTPATTERN.NONE;
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
        return outake.shootLoaded();
    }

    private Command shootGPP() {
        return new SequentialCommandGroup(
                shootGreen(),
                shootPurple(),
                shootLoaded()
        );
    }
    private Command shootPGP() {
        return new SequentialCommandGroup(
                shootPurple(),
                shootGreen(),
                shootLoaded()
        );
    }
    private Command shootPPG() {
        return new SequentialCommandGroup(
                shootPurple(),
                shootPurple(),
                shootLoaded()
        );
    }
    public Command shootPattern() {
        return new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(ARTIFACTPATTERN.GPP,shootGPP());
                    put(ARTIFACTPATTERN.PGP,shootPGP());
                    put(ARTIFACTPATTERN.PPG,shootPPG());
                    put(ARTIFACTPATTERN.NONE, new WaitCommand(1));
                }},
                () -> pattern);
    }

    @Override
    public void run() {
        if (camera != null && pattern == ARTIFACTPATTERN.NONE){
            for (AprilTagDetection detection : camera.currentDetections) {
                if (detection.id == ARTIFACTPATTERN.GPP.getPattern()) {
                    pattern = ARTIFACTPATTERN.GPP;
                    break;
                } else if (detection.id == ARTIFACTPATTERN.PGP.getPattern()) {
                    pattern = ARTIFACTPATTERN.PGP;
                    break;
                } else if (detection.id == ARTIFACTPATTERN.PPG.getPattern()) {
                    pattern = ARTIFACTPATTERN.PPG;
                    break;
                }
            }
        }

        super.run();
    }
}
