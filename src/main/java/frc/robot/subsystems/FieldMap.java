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

    public Pose2d getBestTagPose(int tagID, int slot, double offset)
    {
        Pose2d tagPose = fieldMap.getTagPose(tagID).get().toPose2d();
        double rotation = Math.PI;
        double xCoord = 0.5;
        double yCoord = 0;

        if(slot == 1) //center
        {
            yCoord = 0;
        }
        else if(slot == 0) //left
        {
            yCoord = -offset;
        }
        else if(slot == 2) //right
        {
            yCoord = offset;
        }

        Transform2d robotToReef = new Transform2d(xCoord, yCoord, new Rotation2d(rotation));

        SmartDashboard.putNumber("FieldMap/TargetTagID", tagID);
        SmartDashboard.putNumber("FieldMap/Transform X", xCoord);
        SmartDashboard.putNumber("FieldMap/Transform Y", yCoord);
        SmartDashboard.putNumber("FieldMao/TargetTagRotation", rotation);

        return tagPose.plus(robotToReef);
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
