package org.firstinspires.ftc.teamcode.opcodes;


import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.A;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.B;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.LEFT_BUMPER;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.ButtonReader;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    GamepadEx controller;
    ButtonReader lB;
    Follower follower;

    Intake intake;
    Outake outake;
    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();
        follower.update();
        controller = new GamepadEx(gamepad1);
        lB = new ButtonReader(controller,GamepadKeys.Button.LEFT_BUMPER);
        follower.startTeleopDrive();
        intake = new Intake(hardwareMap);
        outake = new Outake(hardwareMap);
        controller.getGamepadButton(A).whenPressed(

                new SequentialCommandGroup(
                        new InstantCommand(() -> outake.SettriggerL(Outake.triggerPosition)),
                        new WaitCommand(100),
                        new InstantCommand(() -> outake.SettriggerL(Outake.resetPosition))
                ));
        controller.getGamepadButton(B).whenPressed(

        new SequentialCommandGroup(
                new InstantCommand(() -> outake.SettriggerR(Outake.triggerPosition)),
                new WaitCommand(100),
                new InstantCommand(() -> outake.SettriggerR(Outake.resetPosition))
        ));

        controller.getGamepadButton(LEFT_BUMPER).whenPressed(InstantCommand(() -> outake.StartOutake()))
                .whenReleased(InstantCommand(() -> outake.StopOutake()));

    }


    @Override
    public void run() {
        super.run();
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        if (lB.wasJustPressed()){
            intake.ToggleIntake();
        }

    }
}