// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

/** Helper methods for the field :) */
public class FieldHelpers {
    private static AprilTagFieldLayout aprilTags = FieldConstants.aprilTags;

    /**
     * This hashmap maps tag numbers to the reef side they are on.<br>
     * </br>
     * key: tag number<br>
     * </br>
     * value: reef side number
     */
    public static HashMap<Integer, Integer> tagNoToReefSideRed = new HashMap<>() {
        {
            put(7, 1);
            put(8, 2);
            put(9, 3);
            put(10, 4);
            put(11, 5);
            put(6, 6);
        }
    };
    public static HashMap<Integer, Integer> tagNoToReefSideBlue = new HashMap<>() {
        {
            put(18, 1);
            put(17, 2);
            put(22, 3);
            put(21, 4);
            put(20, 5);
            put(19, 6);
        }
    };

    /** Decides which alliance's apriltags to use. */
    public static HashMap<Integer, Integer> tagNoToReefSide() {
        return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? tagNoToReefSideBlue
                : tagNoToReefSideRed;
    }

    /**
     * @param robotPose the pose of the robot
     * @return the reef side number that the robot is closest to
     */
    public static int findNearestReefSide(Pose2d robotPose) { // TODO: Pose2d.nearest() exists...
        Translation2d robotTranslation = robotPose.getTranslation();
        double minDistance = Double.MAX_VALUE;
        int nearestReefSide = -1;

        for (int tagID : tagNoToReefSide().keySet()) {
            Pose2d tagPose = aprilTags.getTagPose(tagID).get().toPose2d();
            Translation2d tagTranslation = tagPose.getTranslation();
            double distance = tagTranslation.getDistance(robotTranslation);
            if (distance < minDistance) {
                minDistance = distance;
                nearestReefSide = tagNoToReefSide().get(tagID);
            }
        }
        return nearestReefSide;
    }

    public static int findNearestReefAprilTag(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        double minDistance = Double.MAX_VALUE;
        int nearestTag = -1;

        for (int tagID : tagNoToReefSide().keySet()) {
            Pose2d tagPose = aprilTags.getTagPose(tagID).get().toPose2d();
            Translation2d tagTranslation = tagPose.getTranslation();
            double distance = tagTranslation.getDistance(robotTranslation);
            if (distance < minDistance) {
                minDistance = distance;
                nearestTag = tagID;
            }
        }
        return nearestTag;
    }

    /**
     * Returns the pose that the robot should pathfind to for a particular reef side
     * on the left or right. Reef side can be returned by findNearestReefSide
     * 
     * @param robotPose   current pose of the robot, used to determine closest reef
     *                    side
     * @param isRightSide is true if we're on the right side of the specified reef
     *                    side
     * @return a Pose2d representing the location and orientation of the robot if
     *         facing the reef on the specified
     */
    public static Pose2d reefLocation(Pose2d robotPose, BooleanSupplier isRightSideSupplier) {
        Pose2d pose = aprilTags.getTagPose(findNearestReefAprilTag(robotPose)).get().toPose2d();
        pose = pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));

        // back up pose by 16" so it's not overlapping the reef
        pose = pose.transformBy(
                new Transform2d(new Translation2d(-FieldConstants.reefLocationBackupDistance,
                        (isRightSideSupplier.getAsBoolean() ? FieldConstants.reefLocationRightDistance : FieldConstants.reefLocationLeftDistance)),
                        new Rotation2d()));

        return pose;
    }

    // public static Pose2d getNearestCoralStation(Pose2d currentPose, boolean isRedAlliance) {
    //     if (!isRedAlliance){
    //         if (currentPose.getY() < (FieldConstants.fieldWidth/2)){
    //             return FieldConstants.coralStationBottomPos;
    //         }else{
    //             return FieldConstants.coralStationTopPos;
    //         }
    //     }else{
    //         if (currentPose.getY() < (FieldConstants.fieldWidth/2)){
    //             return FieldConstants.coralStationTopPos;
    //         }else{
    //             return FieldConstants.coralStationBottomPos;
    //         }
    //     }
    // }
    public static Pose2d getNearestCoralStation(Pose2d currentPose, boolean isRedAlliance) {
        if (!isRedAlliance){
            if (currentPose.getY() < (FieldConstants.fieldWidth/2)){
                Pose2d pose = aprilTags.getTagPose(12).get().toPose2d();
                pose = pose.transformBy(new Transform2d(new Translation2d(0.315, 0.315), new Rotation2d()));
                return pose;
            }else{
                Pose2d pose = aprilTags.getTagPose(13).get().toPose2d();
                pose = pose.transformBy(new Transform2d(new Translation2d(0.315, -0.315), new Rotation2d()));
                return pose;
            }
        }else{
            // if (currentPose.getY() < (FieldConstants.fieldWidth/2)){
            //     Pose2d pose = aprilTags.getTagPose(1).get().toPose2d();
            //     pose = pose.transformBy(new Transform2d(new Translation2d(-0.315, 0.315), new Rotation2d()));
            //     return pose;
            // }else{
            //     Pose2d pose = aprilTags.getTagPose(2).get().toPose2d();
            //     pose = pose.transformBy(new Transform2d(new Translation2d(-0.315, -0.315), new Rotation2d()));
            //     return pose;
            // }
        if (currentPose.getY() > (FieldConstants.fieldWidth/2)){
                Pose2d pose = aprilTags.getTagPose(12).get().toPose2d();
                pose = pose.transformBy(new Transform2d(new Translation2d(0.315, 0.315), new Rotation2d()));
                return pose;
            }else{
                Pose2d pose = aprilTags.getTagPose(13).get().toPose2d();
                pose = pose.transformBy(new Transform2d(new Translation2d(0.315, -0.315), new Rotation2d()));
                return pose;
            }
        }
    }
}
