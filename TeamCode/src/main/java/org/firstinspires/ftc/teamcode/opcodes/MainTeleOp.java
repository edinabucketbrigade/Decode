package org.firstinspires.ftc.teamcode.opcodes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;

import java.util.List;

@Configurable
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    private GamepadEx controller;
    private Follower follower;
    private List<LynxModule> hubs;

    private BucketRobot robot;

    public static Pose starting = new Pose(56.000, 8.000, Math.toRadians(90.0));
    Trigger left_trigger, right_trigger;

    private boolean disableDrivetrain = false;
    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();

        if (!disableDrivetrain) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(starting);

            follower.update();
            follower.startTeleopDrive();
        }

        robot = new BucketRobot(hardwareMap, telemetry, follower);
        controller = new GamepadEx(gamepad1);

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        left_trigger = new Trigger(() -> controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2).whenActive(
                new InstantCommand(() -> robot.shootLeft())
        );
        right_trigger = new Trigger(() -> controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2).whenActive(
                new InstantCommand(() -> robot.shootRight())
        );


        // DPAD_LEFT and resets left servo
        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                robot.shootLeft()
        );
        // DPAD_RIGHT triggers and resets right servo
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                robot.shootRight()
        );


        // LEFT_BUMPER controlls the start and stop of the outake
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(robot.enableIntake(), robot.disableIntake());
        // RIGHT_BUMPER controlls the start and stop of the intake
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(robot.enableOutake(), robot.disableOutake());
        // A shoots the green ball
        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robot.shootGreen());
        // B shoots the purple ball
        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(robot.shootPurple());
        // X shoots the collum with a ball in it (shoots loaded)
        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(robot.shootLoaded());

    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);

        super.run();
        robot.run();

        if (!disableDrivetrain) {
            follower.update();
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            telemetry.addData("Pose", "<%f,%f>:%f",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));
        }

        telemetry.update();
    }
}