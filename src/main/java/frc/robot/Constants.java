// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    private Constants() { throw new UnsupportedOperationException(); }

    public final class CanIds {
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
        public int getLeftDriveLeaderId() { return 13; }
        public int getLeftDriveFollowerId() { return 14; }
        public int getRightDriveLeaderId() { return 15; }
        public int getRightDriveFollowerId() { return 16; }
    }

    public final class PneumaticPortIds {
        public int getIntakeId() { return 0; }
        public int getShifterId() { return 1; }
    }
}
