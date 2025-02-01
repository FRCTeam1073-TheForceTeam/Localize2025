package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Path.PathFeedback;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public class DrivePath extends Command
{
  /** Creates a new DriveThroughTrajectory. */

  double distanceTolerance = 0.1;
  double angleTolerance = 0.1;

  Drivetrain drivetrain;
  Pose2d robotPose;
  Path path;
  Localizer localizer;
  int currentSegmentIndex = -1;

  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double currentTime;
  double maxVelocity;
  double maxAngularVelocity;
  double maxAcceleration;
  double endTime;
  double xVelocity;
  double yVelocity;
  double thetaVelocity;
  double startTime;

 /**
  * Constructs a schema to drive along a given path.
  * @param ds
  * @param path
  */
  public DrivePath(Drivetrain ds, Path path, Localizer localizer) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    this.path = path;
    this.localizer = localizer;

    xController = new PIDController(
      1.1, 
      0.0, 
      0.01
    );

    yController = new PIDController(
      1.1, 
      0.0, 
      0.01
    );

    thetaController = new PIDController(
      1.2, 
      0.0,
      0.01
    );
    SmartDashboard.putString("DrivePath/Status","Idle");
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    startTime = Timer.getFPGATimestamp();
    currentTime = 0.01;
    // currentSegmentIndex = path.closestSegment(drivetrain.getOdometry());   
    currentSegmentIndex = 0; 

    if (currentSegmentIndex != -1 && path.segments.get(currentSegmentIndex).entryCommand != null) 
    {
      CommandScheduler.getInstance().schedule(path.segments.get(currentSegmentIndex).entryCommand);
    }

    if (currentSegmentIndex != -1 && path.segments.get(currentSegmentIndex).entryActivate != null)
    {
      path.segments.get(currentSegmentIndex).entryActivate.activate(path.segments.get(currentSegmentIndex).entryActivateValue);
    }

    xController.reset();
    yController.reset();
    thetaController.reset();
    SmartDashboard.putString("DrivePath/Status",String.format("Starting Segment: %d", currentSegmentIndex));
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() 
  {
    if (currentSegmentIndex < 0) 
    {
      SmartDashboard.putString("DrivePath/Status","Invalid segment index.");

      // System.out.println("DrivePathSchema: Invalid segment index.");
      // Stop:
      //TODO: send command to the drivetrain
      return; // Don't run.
    }

    currentTime = Timer.getFPGATimestamp() - startTime;
    //robotPose = drivetrain.getOdometry();
    robotPose = localizer.getPose();
    
    // Compute position and velocity desired from where we actually are:
    PathFeedback pathFeedback = path.getPathFeedback(currentSegmentIndex, robotPose);

    // maxVelocity = pathFeedback.velocity.norm();
    // maxAngularVelocity = pathFeedback.velocity.norm();

    maxVelocity = path.segments.get(currentSegmentIndex).velocity;
    maxAngularVelocity = 1;
    

    if (currentSegmentIndex >= path.segments.size() - 1)
    {
      // Last point is meant to be a bit different:
      //xVelocity = xController.calculate(robotPose.getX(), pathFeedback.pose.getX()) + 0.3 * pathFeedback.velocity.get(0,0);
      //yVelocity = yController.calculate(robotPose.getY(), pathFeedback.pose.getY()) + 0.3 * pathFeedback.velocity.get(1,0);

      xVelocity = pathFeedback.velocity.get(0,0);
      yVelocity = pathFeedback.velocity.get(1,0);
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), path.finalOrientation);
    }
    else
    {
      //xVelocity = xController.calculate(robotPose.getX(), pathFeedback.pose.getX()) + 0.3 * pathFeedback.velocity.get(0,0);
      //yVelocity = yController.calculate(robotPose.getY(), pathFeedback.pose.getY()) + 0.3 * pathFeedback.velocity.get(1,0);
      xVelocity = pathFeedback.velocity.get(0,0);
      yVelocity = pathFeedback.velocity.get(1,0);
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), path.getPathOrientation(currentSegmentIndex, robotPose)); 
    }
    
    // Clamp to maximums for safety:
    xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    thetaVelocity = MathUtil.clamp(thetaVelocity, -maxAngularVelocity, maxAngularVelocity);

    // Create (field-centric) chassis speeds:
    ChassisSpeeds fcSpeeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    // SmartDashboard.putNumber("Trajectory Time", currentTime);

    SmartDashboard.putNumber("DrivePath/TrajFBVx", pathFeedback.velocity.get(0,0));
    SmartDashboard.putNumber("DrivePath/TrajFBVy", pathFeedback.velocity.get(1,0));
    SmartDashboard.putNumber("DrivePath/MaxVelocity", maxVelocity);

    SmartDashboard.putNumber("DrivePath/TrajVx", fcSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DrivePath/TrajVy", fcSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DrivePath/TrajW", fcSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putString("DrivePath/Status", String.format("Segment Index: %d", currentSegmentIndex));
    SmartDashboard.putNumber("DrivePath/SegmentsSize", path.segments.size());

    // Controlled drive command with weights from our path segment feedback, set our two channels of schema output/w weights.
    drivetrain.setTargetChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity, 
                    yVelocity,
                    thetaVelocity, 
                    Rotation2d.fromDegrees(localizer.getPose().getRotation().getDegrees()) // gets fused heading
                )
            );
    //drivetrain.setTargetChassisSpeeds(fcSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Set our schema output to full stop
    drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

  if (currentSegmentIndex < 0) 
  {
    return true; // Finished if we don't have a good index.
  }
  

  // Otherwise check our segment, and manage command launch as we move along segments.
  Path.Segment seg = path.segments.get(currentSegmentIndex);
  if (path.atEndPoint(currentSegmentIndex, localizer.getPose())) 
  {

    // Cancel entry comamnd for this segment:
    if (seg.entryCommand != null) 
    {
      CommandScheduler.getInstance().cancel(seg.entryCommand);
    }
    // Kick off our exit command for this segment:
    if (seg.exitCommand != null) 
    {
      CommandScheduler.getInstance().schedule(seg.exitCommand);
    }

    if (seg.entryActivate != null)
    {
      seg.entryActivate.activate(seg.entryActivateValue);
    }

    if (seg.exitActivate != null)
    {
      seg.exitActivate.activate(seg.exitActivateValue);
    }

    // Move to next path segment:
    currentSegmentIndex = currentSegmentIndex + 1;
    if (currentSegmentIndex >= path.segments.size()) 
    {
      SmartDashboard.putString("DrivePath/Status","Finished.");
      return true;
    } 
    else 
    {
      // Move to new segment:
      seg = path.segments.get(currentSegmentIndex);
      if (seg.entryCommand != null) 
      {
        CommandScheduler.getInstance().schedule(seg.entryCommand);
      }

      if (seg.entryActivate != null)
      {
        seg.entryActivate.activate(seg.entryActivateValue);
      }
    }
  }
  return false; // Keep on going.
}

}