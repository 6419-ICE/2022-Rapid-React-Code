// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Constants for the drive train
    public static class DrivetrainConstants{
        //Universal motor speed limit
        public static final double speedLmt = .4;

        //Drive train encoder values
        public static final double gearboxRatio = 1/10.71; // Input divided by output
        public static final double ticksPerRotation = 2048 / gearboxRatio; // Motor CPR times gearbox ratio
        public static final double wheelDiameter = 6;
        public static final double circumference = wheelDiameter * Math.PI;
        public static final double rotationsPerInch = 1 / (circumference);
        //public static final double distanceCorrector = 1.6;
        public static final double ticksPerInch = ticksPerRotation * rotationsPerInch;

        //Motor ports
        public static final int 
            LEFT_FRONT_PIN = 1,
            LEFT_BACK_PIN = 2,
            RIGHT_FRONT_PIN = 3,
            RIGHT_BACK_PIN = 4;
    }

    public static class IntakeConstants{
        public static final int
            INTAKE_ARM_PIN = 5,
            INTAKE_MOTOR_PIN = 6;
        
        public static final int
            H_EFFECT_PORT = 0;

        public static final double
            armSpdLmt = .3,
            motorSpdLmt = .7;
    }

    public static class UptakeConstants {
        public static final int
            UPTAKE_MOTOR_PIN = 7,
            LOADER_MOTOR_PIN = 8;

        public static final int
            LOAD_SENSOR_PORT = 1;

        public static final double
            uptakeMotorLmt = .9,
            loadMotorLmt = .4;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR_PIN = 9;

        public static final double SHOOTER_HIGH_FIRING_SPEED = 12000.0;
        public static final double SHOOTER_LOW_FIRING_SPEED = 6000.0;

        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;

        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.047;

    }

    public static class HangerConstants {
        public static final int BACK_CLIMBER_ONE_PIN = 10;
        public static final int BACK_CLIMBER_TWO_PIN = 11;

        public static final double hangerSpeedLmt = .8;
    }

    public static final int joy1 = 0, buttonBox = 1, joy3 = 2;

    public static final int gamepadJoy = 0;

    	// Gamepad axis
    public static class gamepadConstants{
        public static final int kGamepadAxisLeftStickX = 0;
        public static final int kGamepadAxisLeftStickY = 1;
        public static final int kGamepadAxisShoulderLeft = 2;
        public static final int kGamepadAxisShoulderRight = 3;
        public static final int kGamepadAxisRightStickX = 4;
        public static final int kGamepadAxisRightStickY = 5;
        

        // Gamepad 2 buttons
        public static final int runIntakeButton = 1; // Bottom Button
        public static final int runUptakeButton = 2; // Right Button
        public static final int button3 = 3; // Left Button
        public static final int button4 = 4; // Top Button
        public static final int button5 = 5;
        public static final int reverseIntakeButton = 6;
        public static final int reverseUptakeButton = 7;
        public static final int button8 = 8;
        public static final int shooterLowButton = 9;
        public static final int shooterHighButton = 10;
        public static final int kGamepadButtonMode = -1;
        public static final int kGamepadButtonLogitech = -1;
        public static final int moveIntakeAxis = 0;
        public static final int moveHangerAxis = 1;

        // Gamepad 2 axis 
        //public static final int moveIntakeAxis = 1;
    }


}
