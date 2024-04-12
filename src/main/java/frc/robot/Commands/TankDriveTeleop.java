package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TankDrive;
import static frc.robot.Utility.Constants.DriveConstants.*;

/*
 * Subclassing command gives you a constructor and 4 lifecycle methods
 * 
 * Initialize runs once when the command is started
 * Execute runs constantly (every 20ms) while the command is running
 * isFinished runs constantly while the command is running. When isFinished returns true, the command ends
 * End runs once at the end of the command. If end runs as a result of isFinished, the parameter will be true. If the command was interrupted, the parameter will be false
 * 
 */
public class TankDriveTeleop extends Command
{
    TankDrive m_TankDrive;

    SlewRateLimiter fwdRateLimiter;
    SlewRateLimiter turnRateLimiter;

    DoubleSupplier fwdStickSupplier;
    DoubleSupplier turnStickSupplier;

    double fwdStick;
    double turnStick;


    public TankDriveTeleop(TankDrive m_TankDrive, DoubleSupplier fwdStickSupplier, DoubleSupplier turnStickSupplier)
    {
        this.m_TankDrive = m_TankDrive; //Make sure the global is the same as the passed parameter
        this.fwdStickSupplier = fwdStickSupplier;
        this.turnStickSupplier = turnStickSupplier;

        fwdRateLimiter = new SlewRateLimiter(maxSpeedMetersPerSecond*2);
        turnRateLimiter = new SlewRateLimiter(maxTurnSpeedRadiansPerSecond*2);

        addRequirements(m_TankDrive);
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute() 
    {
        turnStick = turnStickSupplier.getAsDouble();
        fwdStick = fwdStickSupplier.getAsDouble();

        turnStick = (Math.abs(turnStick) < turnDeadband) ? 0 : turnStick;
        fwdStick = (Math.abs(fwdStick) < fwdDeadband) ? 0 : fwdStick;

        turnStick = turnStick*turnStick*(turnStick < 0 ? -1 : 1);
        fwdStick = fwdStick*fwdStick*(fwdStick < 0 ? -1 : 1);

        turnStick *= maxTurnSpeedRadiansPerSecond;
        fwdStick *= maxSpeedMetersPerSecond;

        turnStick = turnRateLimiter.calculate(turnStick);
        fwdStick = fwdRateLimiter.calculate(fwdStick);



    }

    //never finished
    @Override
    public boolean isFinished() 
    {
        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        
    }

}
