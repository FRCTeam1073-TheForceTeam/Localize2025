// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.FieldMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  Drivetrain drivetrain;
  AprilTagFinder tagFinder;
  Localizer localizer;
  FieldMap fieldMap;

  //Map<Integer, Double> tagThetas = new HashMap<>();


  /** Creates a new alignToTag. */
  public AlignToTag(Drivetrain drivetrain, AprilTagFinder tagFinder, Localizer localizer, FieldMap fieldMap) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.tagFinder = tagFinder;
    this.localizer = localizer;
    this.fieldMap = fieldMap;
    // tagThetas.put(11, Math.PI / 4); // probably not relevant
    // tagThetas.put(10, 0.0);
    // tagThetas.put(9, - Math.PI / 4);
    // tagThetas.put(8, - 3 * Math.PI / 4);
    // tagThetas.put(7, Math.PI);
    // tagThetas.put(6, Math.PI / 4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public Command aprilTagAlign(PhotonTrackedTarget aprilTag, String coralSide) 
  {
    Pose2d currentPose = localizer.getPose();
    Optional<Pose3d> tagPose = FieldMap.fieldMap.getTagPose(aprilTag.getFiducialId());
    if(!tagPose.isPresent()) 
    {
      // it doesn't have a pose to give us yet, bail
      return null; //TODO: handle null vales
    }
    
    Pose3d phyTagPos = tagPose.get();
    //double theta = (tagThetas.get(aprilTag.getFiducialId()) != null) ? tagThetas.get(aprilTag.getFiducialId()) : 0;
    double theta = (phyTagPos.getRotation() != null) ? phyTagPos.getRotation().getZ() : 0; // TODO: check if Z is the right rotation to get
    Point start = new Point(currentPose.getX(), currentPose.getY());
    Point destination = new Point(phyTagPos.getX() - 0.35 * Math.cos(theta), phyTagPos.getY() - 0.35 * Math.sin(theta));

    ArrayList<Segment> segment = new ArrayList<Segment>();
    segment.add(new Segment(start, destination, theta, 0.5));

    Path path = new Path(segment, theta);
    return new SequentialCommandGroup(new DrivePath(drivetrain, path, localizer));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
