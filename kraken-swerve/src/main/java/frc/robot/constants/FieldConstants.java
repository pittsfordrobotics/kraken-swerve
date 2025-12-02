// Thank You 6328 Mechanical Advantage for the field constants!!!
package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2025ReefscapeWelded;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets of corners start in the lower left moving clockwise.
 * All units in meters.

 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE ALLIANCE wall.
 *
 * Length refers to the x direction (as described by wpilib)
 * Width refers to the y direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(690.876);
  public static double fieldWidth = Units.inchesToMeters(317);

  public static double reefLocationBackupDistance = Units.inchesToMeters(18);
  public static double reefLocationLeftDistance = Units.inchesToMeters(11);
  public static double reefLocationRightDistance = Units.inchesToMeters(-4);
  public static double startingLineX = Units.inchesToMeters(311.5);
  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  // public static Rotation2d coralStationBottomRot = Rotation2d.fromDegrees(54);
  // public static Pose2d coralStationBottomPos = new Pose2d(0.96, 1.4, coralStationBottomRot);
  // public static Pose2d coralStationBottomPos = new Pose2d(0, 10, coralStationBottomRot);
  // public static Rotation2d coralStationTopRot = Rotation2d.fromDegrees(-54);
  // public static Pose2d coralStationTopPos = new Pose2d(1.116, 7.246, coralStationBottomRot);
  // public static Pose2d coralStationTopPos = new Pose2d(0, 10, coralStationBottomRot);
  public static Rotation2d coralStationBottomRot = new Rotation2d(45);
  public static Pose2d coralStationBottomPos = new Pose2d(1.139, 1.154, coralStationBottomRot);
  public static Rotation2d coralStationTopRot = new Rotation2d(-45);
  public static Pose2d coralStationTopPos = new Pose2d(1.295, 7, coralStationBottomRot);

  public static Rotation2d algaeProcessorRot = Rotation2d.fromDegrees(-90);
  public static Pose2d algaeProcessorPos = new Pose2d(5.988, 0.41, algaeProcessorRot);

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}