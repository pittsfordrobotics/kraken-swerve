package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import frc.robot.lib.SwerveModuleConstants;

public class SwerveConstants {
    private static SwerveModuleConstants FRONT_LEFT_CONSTANTS = new SwerveModuleConstants(0.157527, 2.0141, 0.14898, 0, 0, 0);
    private static SwerveModuleConstants FRONT_RIGHT_CONSTANTS = new SwerveModuleConstants(0.14548, 1.9997, 0.18201, 0, 0, 0);
    private static SwerveModuleConstants BACK_LEFT_CONSTANTS = new SwerveModuleConstants(0.15314, 1.9436, 0.1457, 0, 0, 0);
    private static SwerveModuleConstants BACK_RIGHT_CONSTANTS = new SwerveModuleConstants(0.14703, 2.0077, 0.18546, 0, 0, 0);
    public static final SwerveModuleConstants[] MODULE_CONSTANTS = {FRONT_LEFT_CONSTANTS, FRONT_RIGHT_CONSTANTS, BACK_LEFT_CONSTANTS, BACK_RIGHT_CONSTANTS};

    public static final double SWERVE_MAXIMUM_VELOCITY = 5.5;
    public static final double SWERVE_MAXIMUM_ANGULAR_VELOCITY = 13.6;

    public static final int FRONT_LEFT_MODULE_INDEX = 0;
    public static final int FRONT_RIGHT_MODULE_INDEX = 1;
    public static final int BACK_LEFT_MODULE_INDEX = 2;
    public static final int BACK_RIGHT_MODULE_INDEX = 3;

    public static final double NOMINAL_VOLTAGE = 12;

    public static final double AUTOBUILDER_MAX_CORAL_VELOCITY = 3;
    public static final double AUTOBUILDER_MAX_CORAL_ANGULAR_VELOCITY = 7;

    public static final double AUTOBUILDER_MAX_CORAL_ACCELERATION = 1;
    public static final double AUTOBUILDER_MAX_CORAL_ANGULAR_ACCELERATION = 2;

    public static final double AUTOBUILDER_MAX_VELOCITY = 3;
    public static final double AUTOBUILDER_MAX_ANGULAR_VELOCITY = 7;

    public static final double AUTOBUILDER_MAX_ACCELERATION = 1;
    public static final double AUTOBUILDER_MAX_ANGULAR_ACCELERATION = 2;

    public static final double AUTOBUILDER_P = 5;
    public static final double AUTOBUILDER_I = 0;
    public static final double AUTOBUILDER_D = 0;

    public static final PIDConstants AUTOBUILDER_XY_PID = new PIDConstants(AUTOBUILDER_P, AUTOBUILDER_I, AUTOBUILDER_D);
    public static final PIDConstants AUTOBUILDER_THETA_PID = new PIDConstants(AUTOBUILDER_P, AUTOBUILDER_I, AUTOBUILDER_D);
}
