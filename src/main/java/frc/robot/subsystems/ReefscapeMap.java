package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;

public class ReefscapeMap implements MapInterface
{
    ArrayList<Pose3d> aprilTagLandmarks = new ArrayList<Pose3d>();
    

    @Override
    public Pose3d getLandmark(int id) 
    {
        return null;
    }

    @Override
    public Pose3d getApriltagLandmark(int id) 
    {
        return aprilTagLandmarks.get(id);
    }
    
}
