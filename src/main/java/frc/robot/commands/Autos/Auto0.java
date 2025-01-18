// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Auto0 {
    public static Command create(AprilTagFinder m_aprilTagFinder, Field2d m_field, Drivetrain m_drivetrain){ 
        boolean isRed;
        double centerY = 4.026;
        double centerX = 8.774;
        double startLineOffset = 12.227 -8.774 - 2.24;//id 10 x value - center x value - offset from reef to startline
        SmartDashboard.putString("Alliance", "None");
        if (m_aprilTagFinder != null)
        {
            if(DriverStation.getAlliance().isPresent())
            {
                if (DriverStation.getAlliance().get() == Alliance.Blue) //TODO: this should be one if statement (fix)
                {
                Pose2d startPos = new Pose2d(centerX-startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
                Pose2d endPos = new Pose2d(5.321, 4.026, new Rotation2d(0)); // ID 21
                ArrayList<Pose2d> hereAndThere = new ArrayList<Pose2d>();
                hereAndThere.add(startPos);
                hereAndThere.add(endPos);
                Trajectory allOverThePlace = TrajectoryGenerator.generateTrajectory(
                                            hereAndThere, 
                                            new TrajectoryConfig(99, 99));
                m_field.getObject("trajectory").setTrajectory(allOverThePlace);
                m_drivetrain.resetOdometry(endPos);
                }
                else if (DriverStation.getAlliance().get() == Alliance.Red)
                {
                SmartDashboard.putString("Alliance", "Red");
                isRed = true;
                Pose2d startPos = new Pose2d(centerX + startLineOffset, centerY, new Rotation2d(Math.PI)); //start at start line
                ///Pose2d there = new Pose2d(2, 4, new Rotation2d(Math.PI)); // #ID we made it up
                //Pose2d endPos = new Pose2d(12.227, 4.026, new Rotation2d(Math.PI)); // ID 10
                Pose2d endPos = new Pose2d(12, 4.03, new Rotation2d(Math.PI)); // close to ID 10 but decimals to the thousandth place cause an error for some reason
                ArrayList<Pose2d> hereAndThere = new ArrayList<Pose2d>();
                hereAndThere.add(startPos);
                hereAndThere.add(endPos);
                Trajectory allOverThePlace = TrajectoryGenerator.generateTrajectory(
                                            hereAndThere, 
                                            new TrajectoryConfig(99, 99));
                m_field.getObject("trajectory").setTrajectory(allOverThePlace);
                m_drivetrain.resetOdometry(endPos);
                }
                else
                {
                SmartDashboard.putString("Alliance", "Null");
                isRed = false;
                Pose2d where = new Pose2d(0, 0, new Rotation2d(0));
                m_drivetrain.resetOdometry(where);
                }
            }
    
        }
        return new WaitCommand(5);
    }
}

