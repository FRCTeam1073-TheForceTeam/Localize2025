package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SchemaArbiter extends Command
{
    ArrayList<MotionSchema> schema = new ArrayList<MotionSchema>();
    Drivetrain drivetrain;
    MotionSchema.Translate totalTranslate = new MotionSchema.Translate(0, 0, 0);
    MotionSchema.Rotate totalRotate = new MotionSchema.Rotate(0, 0);
    boolean isFieldCentric;
    boolean stopOnEnd;
    boolean auto;

    public SchemaArbiter(Drivetrain drivetrain, boolean stopOnEnd, boolean auto)
    {
        this.drivetrain = drivetrain;
        isFieldCentric = true;
        this.stopOnEnd = stopOnEnd;
        this.auto = auto;
        addRequirements(drivetrain);
    }

    public void setFieldCentric(boolean fieldCentric)
    {
        isFieldCentric = fieldCentric;
    }

    public boolean getFieldCentric()
    {
        return isFieldCentric;
    }

    public void addSchema(MotionSchema schema)
    {
        this.schema.add(schema);
    }

    @Override
    public void initialize()
    {
        for (int i = 0; i < schema.size(); i++)
        {
            schema.get(i).initialize(drivetrain);
        }
    }

    @Override
    public void execute()
    {
        // clear total
        totalTranslate.zero();
        totalRotate.zero();

        if (auto)
        {
            for (int i = 0; i < schema.size(); i++)
            {
                schema.get(i).execute();
            }
        }
        // loop over all schema
        for (int i = 0; i < schema.size(); i++)
        {
            // add them into total
            totalTranslate.accumulate(schema.get(i).getTranslate());
            totalRotate.accumulate(schema.get(i).getRotate());
        }
        // safe divide total weight
        totalTranslate.resolve();
        totalRotate.resolve();

        SmartDashboard.putNumber("Total X Translate", totalTranslate.vx);
        SmartDashboard.putNumber("Total Y Translate", totalTranslate.vy);
        SmartDashboard.putNumber("Total Rotate", totalRotate.omega);

        // send to drive subsystem
        if (isFieldCentric)
        {
            drivetrain.setTargetChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    totalTranslate.vx, 
                    totalTranslate.vy,
                    totalRotate.omega, 
                    Rotation2d.fromDegrees(drivetrain.getHeadingDegrees()) // gets fused heading
                )
            );
        }
        else
        {
            drivetrain.setTargetChassisSpeeds(
                new ChassisSpeeds(
                    totalTranslate.vx, 
                    totalTranslate.vy, 
                    totalRotate.omega
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        if (stopOnEnd)
        {
            drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished()
    {
        //return false;
        for(int i = 0; i < schema.size(); i++)
        {
            if (schema.get(i).isFinished())
            {
                return true;
            }
        }
        return false;
    }
}
