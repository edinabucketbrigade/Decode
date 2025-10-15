package org.firstinspires.ftc.teamcode.opcodes;


import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.A;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();
        follower.update();
        controller = new GamepadEx(gamepad1);
        lB = new ButtonReader(controller,GamepadKeys.Button.LEFT_BUMPER);
        follower.startTeleopDrive();
        intake = new Intake(hardwareMap);
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