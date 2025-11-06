package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.sun.source.tree.IfTree;

import java.util.Set;

@Configurable
public class Outake extends SubsystemBase {
        private MotorEx flywheel;
        private ServoEx triggerL;
        private ServoEx triggerR;
        private SensorColor leftSensor;
        private SensorColor rightSensor;
        private double maxSpeed;

        public static double kP = 20;
        public static double kV = 0.7;
        public static double speed = 1.0;

        public static double resetPosition = 0.4;
        public static double triggerPosition = 1.0;
        public static long triggerDelay = 150;
        public boolean isRunning;

        public Outake(HardwareMap hardwareMap) {

            flywheel = new MotorEx(hardwareMap, "flywheel_outake", Motor.GoBILDA.BARE);
            flywheel.setBuffer(1.0);
            maxSpeed = flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
            flywheel.setRunMode(Motor.RunMode.VelocityControl);
            flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            flywheel.setInverted(true);
            flywheel.setVeloCoefficients(kP, kV, 0);
            isRunning = false;

            triggerL = new ServoEx(hardwareMap,"Servo_Left", 0,1);
            triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
            triggerL.setInverted(true);
            triggerL.set(resetPosition);
            triggerR.set(resetPosition);

            //leftSensor = new SensorColor(hardwareMap, "Sensor_Left");
            //rightSensor = new SensorColor(hardwareMap, "Sensor_Right");
        }

        public void StartOutake() {

            flywheel.setVelocity(speed*maxSpeed);

        }

        public void StopOutake() {

            flywheel.set(0);
        }

        private boolean isFast() {
            return (flywheel.getVelocity()/(speed*maxSpeed)) > 0.9;
        }

        public void SettriggerL(double position){
            triggerL.set(position);
        }
        public void SettriggerR(double position){
            triggerR.set(position);
        }
        public CommandBase shootL(){
           return new SequentialCommandGroup(
                    new WaitUntilCommand(this::isFast),
                    new InstantCommand(() -> SettriggerL(triggerPosition)),
                    new WaitCommand(triggerDelay),
                    new InstantCommand(() -> SettriggerL(resetPosition))
            );

        }
        public CommandBase shootR() {
            return new SequentialCommandGroup(
                    new WaitUntilCommand(this::isFast),
                    new InstantCommand(() -> SettriggerR(triggerPosition)),
                    new WaitCommand(triggerDelay),
                    new InstantCommand(() -> SettriggerR(resetPosition))
            );
        }
    public void ToggleOutake() {
            if (!isRunning) {
                isRunning = true;
                StartOutake();
            } else {
                isRunning = false;
                StopOutake();

            }
        }
    }


