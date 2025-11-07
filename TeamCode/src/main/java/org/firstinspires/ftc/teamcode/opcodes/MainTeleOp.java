package org.firstinspires.ftc.teamcode.opcodes;



import android.hardware.TriggerEventListener;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    private GamepadEx controller;
    private ButtonReader lB;
    private Follower follower;
    private List<LynxModule> hubs;
    TelemetryData telemetryData = new TelemetryData(telemetry);


    private Intake intake;
    private Outake outake;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();
        follower.update();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        controller = new GamepadEx(gamepad1);
        lB = new ButtonReader(controller,GamepadKeys.Button.LEFT_BUMPER);
        follower.startTeleopDrive();
        intake = new Intake(hardwareMap);
        outake = new Outake(hardwareMap);
        // A triggers and resets left servo
        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                outake.shootL()
        );
        // B triggers and resets right servo
        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(

                outake.shootR()
        );

        // LEFT_BUMPER controlls the start and stop of the outake
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> outake.ToggleOutake()));
        // RIGHT_BUMPER controlls the start and stop of the intake
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.ToggleIntake()));

    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}