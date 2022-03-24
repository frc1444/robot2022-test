// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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

    /** Amount of time to debounce input triggers in seconds */
    public static final double INPUT_DEBOUNCE = 0.05;

    public final class InputConstants {

        /** Polynomial curve to apply to drive inputs - a value great than 1 will make the input less sensitive with small inputs
         *  i.e. smoother control when moving slowly
        */
        public static final int FORWARD_INPUT_CURVE = 3;
        public static final int ROTATE_INPUT_CURVE = 3;

        /** Deadzone for drive controllers */
        public static final double DRIVE_JOYSTICK_DEADZONE = 0.05;
    }

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
        public static int INTAKE_FWD = 15;          // orange
        public static int INTAKE_REV = 11;          // black/yellow
        public static int SHIFTER_FWD = 14;         // red
        public static int SHIFTER_REV = 10;         // yellow
        public static int SHOOTER_HOOD_FWD = 13;    // green
        public static int SHOOTER_HOOD_REV = 9;     // black
        public static int CLIMB_FWD = 12;           // blue
        public static int CLIMB_REV = 8;            // no color
    }
    public static final class Controller {
        public static final int PORT_PS4_DRIVER = 0;
        public static final int PORT_PS4_OPERATOR = 4;
    }

    public static final class DigitalIO {
        public static final int LOWER_BALL_SENSOR = 0;
        public static final int UPPER_BALL_SENSOR = 1;
    }

    public static final class DriveConstants {
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KF = 0.06;
    
        public static final double TURN_KP = 0.001;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;

        public static final double STABILIZATION_KP = 0.001;
        public static final double STABILIZATION_KI = 0.0;
        public static final double STABILIZATION_KD = 0.0;

        public static final double TURN_TOLERANCE_DEG = 5;
        public static final double TURN_RATE_TOLERANCE = 10;    // degrees per second

        public static final double RAMP_RATE = 0.4;
        public static final double DEAD_ZONE = 0.04;
    }

    public static final class ShooterConstants {
        public static final double SHOOTER_KP = 0.15;
        public static final double SHOOTER_KF = 0.04;
        public static final double SHOOTER_KI = 0.00005;
        public static final double SHOOTER_KD = 0.1;
        public static final double SHOOTER_RAMP_RATE = 0.4;        
        public static final double SHOOTER_ERROR_LIMIT = 200;

        /** Shooter speed for close shots */
        public static final double SHOOT_CLOSE_SPEED = -0.40;

        /** Shooter speed for far shots */
        public static final double SHOOT_FAR_SPEED = -0.35;

        public static final double HOOD_KP = 0.06;
        public static final double HOOD_KF = 0.02;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.0;

        public static final double HOOD_SETPOINT = -1.0;

        public static final DoubleSolenoid.Value HOOD_LOW = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value HOOD_HIGH = DoubleSolenoid.Value.kReverse;
    }

    public static final class IntakeConstants {
        public static final DoubleSolenoid.Value INTAKE_UP = DoubleSolenoid.Value.kReverse;
        public static final DoubleSolenoid.Value INTAKE_DOWN = DoubleSolenoid.Value.kForward;
        public static final double INTAKE_RAMP_RATE = 0.3;
    }
}
