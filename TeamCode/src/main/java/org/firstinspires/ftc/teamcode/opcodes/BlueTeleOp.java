package org.firstinspires.ftc.teamcode.opcodes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;

@Configurable
@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends CommandOpMode {
    private GamepadEx controller;
    private Follower follower;

    private BucketRobot robot;

    private boolean manualDrive = true;

    public BlueTeleOp(boolean isBlueAlliance) {
        BucketRobot.blueAlliance = isBlueAlliance;
    }

    public BlueTeleOp() {
        this(true);
    }

    ToggleButtonReader slowMode, fixedOutake, turnToTarget, emptyIntake;

    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        //set starting position to far shooting zone or keep the position from auto
        if (BucketRobot.currentPos == null)
            follower.setStartingPose(BucketRobot.createPose(56, 8.5, Math.toRadians(90)));
        else
            follower.setStartingPose(BucketRobot.currentPos);


        robot = new BucketRobot(hardwareMap, telemetry, follower);


        follower.update();
        follower.startTeleopDrive();

        controller = new GamepadEx(gamepad1);

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

        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(robot.shootPattern());


        slowMode = new ToggleButtonReader(
                controller, GamepadKeys.Button.DPAD_DOWN
        );
        fixedOutake = new ToggleButtonReader(
                controller, GamepadKeys.Button.DPAD_UP
        );
        turnToTarget = new ToggleButtonReader(
                controller, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );
        emptyIntake = new ToggleButtonReader(
                controller, GamepadKeys.Button.DPAD_LEFT
        );
    }

    @Override
    public void run() {
        super.run();
        robot.run();

        fixedOutake.readValue();
        slowMode.readValue();
        turnToTarget.readValue();
        emptyIntake.readValue();

        robot.emptyIntake(emptyIntake.getState());

        if (manualDrive) {
            if (slowMode.getState()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/2, true);
            }
            else {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }
        }

        robot.fixedSpeed = fixedOutake.getState();
        robot.turnToTarget = turnToTarget.getState();

        if (manualDrive && robot.turnToTarget)
            manualDrive = false;
        else if (!manualDrive && !robot.turnToTarget){
            manualDrive = true;
            follower.startTeleopDrive();
        }

        if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            robot.shootLeft().schedule();
        }
        if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
            robot.shootRight().schedule();
        }

        telemetry.update();
    }
}