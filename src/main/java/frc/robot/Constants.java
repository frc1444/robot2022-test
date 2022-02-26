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
        public int getPdhId() { return 1; }
        public int getPhId() { return 2; }
        public int getIntakeId() { return 3; }
        public int getLeftSingulateId() { return 4; }
        public int getRightSingulateId() { return 5; }
        public int getLowerIndexId() { return 6; }
        public int getUpperIndexId() { return 7; }
        public int getShooterId() { return 8; }
        public int getFirstStageLeftClimbId() { return 9; }
        public int getFirstStageRightClimbId() { return 10; }
        public int getSecondStageLeftClimbId() { return 11; }
        public int getSecondStageRightClimbId() { return 12; }
//        public int getLeftDriveLeaderId() { return 13; }
        public static int LEFT_DRIVE_LEADER = 13;
//        public int getLeftDriveFollowerId() { return 14; }
        public static int LEFT_DRIVE_FOLLOWER = 14;
        public int getRightDriveLeaderId() { return 15; }
        public static int RIGHT_DRIVE_LEADER = 15;
        public int getRightDriveFollowerId() { return 16; }
        public static int RIGHT_DRIVE_FOLLOWER = 16;
    }

    public static final class PneumaticPortIds {
        public int getIntakeId() { return 0; }
        public int getShifterId() { return 1; }
    }
    public static final class Controller {
        public static final int PORT_PS4 = 0;
    }
}
