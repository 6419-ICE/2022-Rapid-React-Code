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
        public static final double gearboxRatio = 1;
        public static final double inchesPerRotation = 1;

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
            armSpdLmt = .4,
            motorSpdLmt = .7;
    }

    public static final int joy1 = 0, joy2 = 1, joy3 = 2;

    public static final int gamepadJoy = 0;

    	// Gamepad axis
	public static final int kGamepadAxisLeftStickX = 1;
	public static final int kGamepadAxisLeftStickY = 2;
	public static final int kGamepadAxisShoulder = 3;
	public static final int kGamepadAxisRightStickX = 4;
	public static final int kGamepadAxisRightStickY = 5;
	public static final int kGamepadAxisDpad = 6;

	// Gamepad buttons
	public static final int kGamepadButtonA = 1; // Bottom Button
	public static final int kGamepadButtonB = 2; // Right Button
	public static final int kGamepadButtonX = 3; // Left Button
	public static final int kGamepadButtonY = 4; // Top Button
	public static final int kGamepadButtonShoulderL = 5;
	public static final int kGamepadButtonShoulderR = 6;
	public static final int kGamepadButtonBack = 7;
	public static final int kGamepadButtonStart = 8;
	public static final int kGamepadButtonLeftStick = 9;
	public static final int kGamepadButtonRightStick = 10;
	public static final int kGamepadButtonMode = -1;
	public static final int kGamepadButtonLogitech = -1;


}
