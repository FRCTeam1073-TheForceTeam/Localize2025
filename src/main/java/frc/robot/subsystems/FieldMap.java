// MAP: loads the map

package frc.robot.subsystems;


import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FieldMap
{
    public static final AprilTagFieldLayout fieldMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public int getBestAprilTagID(Pose2d robotPose) 
    {
        double shortestDistance = 998;

        List<AprilTag> aprilTags = fieldMap.getTags();
        int bestID = -1;

        for(AprilTag tag : aprilTags) 
        {
            if (findDistance(robotPose, tag.ID) < shortestDistance) 
            {
                shortestDistance = findDistance(robotPose, bestID);
                bestID = tag.ID;
            }
        }
        return bestID;
    }

    public Pose2d getBestTagPose(int tagID, int slot, double distToReef)
    {
        Pose2d tagPose = fieldMap.getTagPose(tagID).get().toPose2d();
        Transform2d offset = null;

        SmartDashboard.putNumber("FieldMap/TargetTag", tagID);



        switch (tagID) 
        {
            case 6:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d();
                if (slot == 2) offset = new Transform2d();
                break;
            case 7:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d();
                if (slot == 2) offset = new Transform2d();
                break;
            case 8:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d();
                if (slot == 2) offset = new Transform2d();
                break;
            case 9:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d(0.25, 0.43, new Rotation2d(5 * Math.PI / 3));
                if (slot == 2) offset = new Transform2d();
                break;
            case 10:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d(0.5, 0, new Rotation2d());
                if (slot == 2) offset = new Transform2d();
                break;
            case 11:
                if (slot == 0) offset = new Transform2d();
                if (slot == 1) offset = new Transform2d(0.25, 0.43, new Rotation2d(Math.PI / 3));
                if (slot == 2) offset = new Transform2d();
                break;
            default:
                return null;
        }
        return tagPose.plus(offset);
    }

    public double findDistance(Pose2d robot2DPose, int tagID) {

        Optional<Pose3d> tag3dPose = FieldMap.fieldMap.getTagPose(tagID);

        if(!tag3dPose.isPresent()) {
            //we don't have a value to work with, bail
            return 999;
        }

        double tagX = tag3dPose.get().getX();
        double tagY = tag3dPose.get().getY();
        double distance = Math.sqrt(Math.pow((tagX - robot2DPose.getX()), 2) + Math.pow((tagY - robot2DPose.getY()), 2));
        return distance;
    }
}
