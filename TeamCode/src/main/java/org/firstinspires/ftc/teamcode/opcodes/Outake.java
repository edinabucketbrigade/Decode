package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
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
        public static double kP = 20;
        public static double kV = 0.7;
        public static double speed = 6000.0;

        public static double resetPosition = .4;
        public static double triggerPosition = .6;
        public boolean isRunning;

        public Outake(HardwareMap hardwareMap) {

            flywheel = new MotorEx(hardwareMap, "flywheel_Main", Motor.GoBILDA.BARE);
            flywheel.setRunMode(Motor.RunMode.VelocityControl);
            flywheel.setVeloCoefficients(kP, kV, 0);
            isRunning = false;

            triggerL = new ServoEx(hardwareMap,"Servo_Left", 0,1);
            triggerR = new ServoEx(hardwareMap, "Servo_Right", 0, 1);
            triggerR.setInverted(true);
        }

        public void StartOutake() {

            flywheel.set(speed);

        }
        @Override
        public void periodic() {
        }
        public void StopOutake() {

            flywheel.set(0);
        }

        pubilc void SettriggerL(double position){
            triggerL.set(position);
        }
        public void SettriggerR(double position){
            triggerR.set(position);
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

)


