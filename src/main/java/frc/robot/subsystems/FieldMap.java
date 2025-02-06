// MAP: loads the map

package frc.robot.subsystems;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class FieldMap
{
    public static final AprilTagFieldLayout fieldMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private Localizer localizer;
    private Pose2d robot2DPose = localizer.getPose();
    //private ArrayList<Double> distances = new ArrayList<Double>();
   
    private List<AprilTag> aprilTags = fieldMap.getTags();
    private AprilTag aprilTag;
    
    // public FieldMap()
    // {

    // }

    public AprilTag getBestAprilTag(Pose2d robot2DPose, List<AprilTag> aprilTags){
        double shortestDistance = 100;

        for(AprilTag tag : aprilTags) {
            if (findDistance(robot2DPose, tag.ID) < shortestDistance)
                aprilTag = tag;
        }

        return aprilTag;
    }

    public double findDistance(Pose2d robot2DPose, int tagID){

        Optional<Pose3d> tag3dPose = fieldMap.getTagPose(tagID);

        if (!tag3dPose.isPresent())
        {
            //we don't have a value to work with, bail
            return -1.0;
        }

        double tagX = tag3dPose.get().getX();
        double tagY = tag3dPose.get().getY();

        double distance = Math.sqrt(Math.pow((tagX - robot2DPose.getX()), 2) + Math.pow((tagY - robot2DPose.getY()), 2));
        return distance;
    }

}
