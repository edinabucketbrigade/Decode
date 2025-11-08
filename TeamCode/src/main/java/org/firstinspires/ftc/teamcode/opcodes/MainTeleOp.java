package org.firstinspires.ftc.teamcode.opcodes;



import android.hardware.TriggerEventListener;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.ButtonReader;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.bylazar.telemetry.JoinedTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    private GamepadEx controller;
    private Follower follower;
    private List<LynxModule> hubs;
    Telemetry bTelemetry;

    private BucketRobot robot;

    @Override
    public void initialize() {
        bTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive();

        robot = new BucketRobot(hardwareMap, bTelemetry);
        controller = new GamepadEx(gamepad1);

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

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
                .whenPressed(robot.toggleIntake());
        // RIGHT_BUMPER controlls the start and stop of the intake
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(robot.toggleOutake());
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
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        bTelemetry.addData("X", follower.getPose().getX());
        bTelemetry.addData("Y", follower.getPose().getY());
        bTelemetry.addData("Heading", follower.getPose().getHeading());
        bTelemetry.update();
    }
}