package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

public class MotionSchema 
{
    public static class Translate
    {
        public double vx;
        public double vy;
        public double weight;


        public Translate(double vx, double vy, double weight)
        {
            this.vx = vx;
            this.vy = vy;
            this.weight = weight;
        }

        public void zero()
        {
            vx = 0;
            vy = 0;
            weight = 0;
        }

        public void accumulate(MotionSchema.Translate newTranslate)
        {
            vx += newTranslate.vx * newTranslate.weight;
            vy += newTranslate.vy * newTranslate.weight;
            weight += newTranslate.weight;
        }

        public void resolve() // only for total schema
        {
            if (weight == 0)
            {
                vx = 0;
                vy = 0;
            }
            else 
            {
                vx /= weight;
                vy /= weight;
            }
        }
    }

    public static class Rotate
    {
        public double omega;
        public double weight;

        public Rotate(double omega, double weight)
        {
            this.omega = omega;
            this.weight = weight;
        }

        public void zero()
        {
            omega = 0;
            weight = 0;
        }

        public void accumulate(MotionSchema.Rotate newRotate)
        {
            omega += newRotate.omega * newRotate.weight;
            weight += newRotate.weight;
        }

        public void resolve() // only for total schema
        {
            if (weight == 0)
            {
                omega = 0;
            }
            else
            {
                omega /= weight;
            }
        }
    }
    
    private MotionSchema.Translate translate = new MotionSchema.Translate(0, 0, 0);
    private MotionSchema.Rotate rotate = new MotionSchema.Rotate(0, 0);

    public MotionSchema() {}

    public final Translate getTranslate()
    {
        return translate;
    }

    public final Rotate getRotate()
    {
        return rotate;
    }

    public void initialize(Drivetrain drivetrain)
    {
        // same thing as update
    }

    public void update(Drivetrain drivetrain)
    {
        // Derived classes override this method and set their translate and rotate terms
    }

    public void execute()
    {

    }

    public void end(boolean interrupted)
    {

    }

    public boolean isFinished()
    {
        return false;
    }

    public final void setTranslate(double vx, double vy, double weight)
    {
        translate.vx = vx;
        translate.vy = vy;
        translate.weight = weight;
    }

    public final void setRotate(double omega, double weight)
    {
        rotate.omega = omega;
        rotate.weight = weight;
    }
}
