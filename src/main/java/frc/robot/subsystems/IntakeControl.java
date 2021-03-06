package frc.robot.subsystems;

// Defines motor percent demands for various functions of the intake subsystem
public final class IntakeControl {
    private IntakeControl() { throw new UnsupportedOperationException(); }

    public static double Stop = 0.0;
    public static double Intake_Intake = 1;
    public static double Intake_Eject = -1;
    public static double Singulate_Intake = 0.45;
    public static double Singulate_Eject = -0.45;
    public static double Upper_Index_Intake = 1.00;
    public static double Upper_Index_Eject = -1.00;
    public static double Lower_Index_Intake = 0.70;
    public static double Lower_Index_Eject = -0.70;
    public static double Upper_Index_Feed = 1.00;
}
