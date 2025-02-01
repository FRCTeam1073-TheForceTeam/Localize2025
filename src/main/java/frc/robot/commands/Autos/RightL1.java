package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public class RightL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, Localizer localizer)  
    {
        Point start = new Point(10, 6);
        Point point1 = new Point(12.5, 4.8);

        ArrayList<Segment> segments = new ArrayList<Segment>();

        segments.add(new Segment(start, point1, 0.86, 1.5));

        Path path = new Path(segments, 0);

        return new DrivePath(drivetrain, path, localizer);
    }  
}
