package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.sun.source.tree.IfTree;

@Configurable
public class Outake extends SubsystemBase {
        private MotorEx flywheel;
        private ServoEx triggerL;
        private ServoEx triggerR;
        public static double kP = 20;
        public static double kV = 0.7;
        public static double speed = 6000.0;
        int a;
        int b;

        public boolean isRunning;

        public Outake(HardwareMap hardwareMap) {

            flywheel = new MotorEx(hardwareMap, "flywheel_Main", Motor.GoBILDA.BARE);
            flywheel.setRunMode(Motor.RunMode.VelocityControl);
            flywheel.setVeloCoefficients(kP, 0, 0);
            flywheel.setFeedforwardCoefficients(0, kV);
            isRunning = false;
            triggerL = new SimpleServo(hardwareMap,"Servo_Left", a, b);
            triggerR = new ServoEx(hardwareMap, "Servo_Right", a, b);

        }

        public void StartOutake() {

            flywheel.set(speed);


        }
        @Override
        public void periodic() {
            if (triggerL.getPosition()<1) {
                triggerL.set(1);
            }else {
                triggerL.set(0);
            }
        }
        public void StopOutake() {

            flywheel.set(0);
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

