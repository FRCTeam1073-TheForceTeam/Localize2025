// MAPDISPLAY: gathers and sends data for the map display, accesses drivetrain and localizer
// 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;

public class MapDisplay extends SubsystemBase{

    private final Field2d field = new Field2d();
    private Drivetrain driveTrain;
    private Localizer localizer;
    private FieldMap fieldMap;
    private Pose3d robotPose;
        
    public MapDisplay(Drivetrain driveTrain, Localizer localizer, FieldMap fieldMap)
    {
        this.driveTrain = driveTrain;
        this.localizer = localizer;
        this.fieldMap = fieldMap;
   
    }

    @Override
    public void periodic()
    { 
        //field.setRobotPose(driveTrain.getOdometry());
        field.setRobotPose(localizer.getPose());
        SmartDashboard.putData("Field", field); //the widget for this is the dropdown named "field"
        //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/smartdashboard/Field2d.html
        //use above to link to look into two robot poses: 1 odometry, 1 localize
    }
}
