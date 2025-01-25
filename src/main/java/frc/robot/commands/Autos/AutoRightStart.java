// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoRightStart 
{
    public static Command create(int level, boolean isRed, Drivetrain drivetrain)
    {
        
        if(isRed == true) {
            if (level == 99) {
                Point point1 = new Point(15.1, 6.039); 
                Point point2 = new Point(15.1, 2.013);
                Point point3 = new Point(8.77, 2.013);
                Point point4 = new Point(8.77, 6.039);
                ArrayList<Segment> segments = new ArrayList<Segment>();
                segments.add(new Segment(point1, point2, Math.PI / 2, 1));
                segments.add(new Segment(point2, point3, Math.PI, 1));
                segments.add(new Segment(point3, point4, 3 * Math.PI / 2, 1));
                segments.add(new Segment(point4, point1, 0, 1));

                Path path = new Path(segments, 0);
                return new SequentialCommandGroup(new DrivePath(drivetrain, path));
            }
        }
        return new WaitCommand(5);
    }
}
