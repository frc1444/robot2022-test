// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    private Constants() { throw new UnsupportedOperationException(); }

    public static final double MAX_FALCON_RPM = 6380;
    /** The number of encoder counts in a single revolution on a Falcon 500 using Talon FX*/
    public static final double FALCON_ENCODER_COUNTS_PER_REVOLUTION = 2048;
    /** Conversion of CTRE units of 100 units/ms*/
    public static final double CTRE_UNIT_CONVERSION = 600;

    /** Talon SRX counts every edge of the quadrature encoder, so 4 * 20 */
    public static final double CIMCODER_COUNTS_PER_REVOLUTION = 80;
    /** A constant needed for some CTRE methods that should just stay 0 */
    public static final int SLOT_INDEX = 0;

    /** Polynomial curve to apply to drive inputs - a value great than 1 will make the input less sensitive with small inputs
     *  i.e. smoother control when moving slowly
    */
    public static final int FORWARD_INPUT_CURVE = 3;
    public static final int ROTATE_INPUT_CURVE = 3;

    /** Deadzone for drive controllers */
    public static final double DRIVE_JOYSTICK_DEADZONE = 0.05;

    /** Shooter speed for high goal */
    public static final double SHOOT_HIGH_SPEED = -0.7;

    /** Shooter speed for low goal */
    public static final double SHOOT_LOW_SPEED = -0.4;

    public static final class CanIds {

        public static int POWER_DIST_HUB = 1;
        public static int PNEUMATIC_HUB = 2;
        public static int INTAKE = 3;
        public static int SINGULATE_LEFT = 4;
        public static int SINGULATE_RIGHT = 5;
        public static int INDEX_LOWER = 6;
        public static int INDEX_UPPER = 7;
        public static int SHOOTER = 8;
        public static int FIRST_STAGE_LEFT_CLIMB = 9;
        public static int FIRST_STAGE_RIGHT_CLIMB = 10;
        public static int SECOND_STAGE_LEFT_CLIMB = 11;
        public static int SECOND_STAGE_RIGHT_CLIMB = 12;
        public static int LEFT_DRIVE_LEADER = 13;
        public static int LEFT_DRIVE_FOLLOWER = 14;
        public static int RIGHT_DRIVE_LEADER = 15;
        public static int RIGHT_DRIVE_FOLLOWER = 16;
        public static int PIGEON_IMU = 17;
        public static int SHOOTER_HOOD = 18;
    }

    public static final class PneumaticPortIds {
        public static int INTAKE_FWD = 15;
        public static int INTAKE_REV = 11;
        public static int SHIFTER_FWD = 14;
        public static int SHIFTER_REV = 10;
        public static int SHOOTER_HOOD_FWD = 13;
        public static int SHOOTER_HOOD_REV = 9;
        public static int CLIMB_FWD = 12;
        public static int CLIMB_REV = 8;
    }
    public static final class Controller {
        public static final int PORT_PS4_DRIVER = 0;
        public static final int PORT_PS4_OPERATOR = 4;
    }

    public static final class DigitalIO {
        public static final int LOWER_BALL_SENSOR = 0;
        public static final int UPPER_BALL_SENSOR = 1;
    }

    /**
     * Button mappings for an Extreme 3D Pro Joystick
     */
    public static final class ControllerExtreme {
        public static int POV = 0;

        public static int GRID_MIDDLE_LEFT = 8;
        public static int GRID_MIDDLE_RIGHT = 9;
    }
}
