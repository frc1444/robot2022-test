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


    public static final class CanIds {
        // Note that as of writing this comment (2022.02.26) only the drive leaders and followers have been programmed

        public int getPdhId() { return 1; }
        public int getPhId() { return 2; }
        public static int INTAKE = 3;
        public static int SINGULATE_LEFT = 4;
        public static int SINGULATE_RIGHT = 5;
        public static int INDEX_LOWER = 6;
        public static int INDEX_UPPER = 7;
        public static int SHOOTER = 8;
        public int getFirstStageLeftClimbId() { return 9; }
        public int getFirstStageRightClimbId() { return 10; }
        public int getSecondStageLeftClimbId() { return 11; }
        public int getSecondStageRightClimbId() { return 12; }
        public static int LEFT_DRIVE_LEADER = 13;
        public static int LEFT_DRIVE_FOLLOWER = 14;
        public static int RIGHT_DRIVE_LEADER = 15;
        public static int RIGHT_DRIVE_FOLLOWER = 16;
    }

    public static final class PneumaticPortIds {
        public int getIntakeId() { return 0; }
        public int getShifterId() { return 1; }
    }
    public static final class Controller {
        public static final int PORT_PS4 = 0;
        public static final int PORT_EXTREME = 1;
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
