package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This subsystem will be used to teach creating inline commands.
 * We will make simple methods in this subsystem (mostly void)
 * We will chain them together in RobotContainer using in-line commands
 */
public class InlineCommands extends SubsystemBase
{
    /*
     * This is an imaginary system that intakes a ball, tilts up, and shoots
     * Here is the robot I'm picturing
     * https://youtu.be/sLlgDgQ-K70?si=_4lUZHxFy0sVkB1Y&t=165
     */
    CANSparkMax intake;
    DigitalInput haveBall;
    Solenoid tilter;
    DoubleSolenoid shooter;

    boolean holdingBall;

    public InlineCommands()
    {
        intake = new CANSparkMax(1, MotorType.kBrushless);
        haveBall = new DigitalInput(0);
        tilter = new Solenoid(PneumaticsModuleType.REVPH, 0);
        shooter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    }

    @Override
    public void periodic() 
    {
        holdingBall = haveBall.get();
    }

    public void intake()
    {
        intake.set(1);
    }

    public void outtake()
    {
        intake.set(-1);
    }

    public void stopIntake()
    {
        intake.set(0);
    }

    public void tiltUp()
    {
        tilter.set(true);
    }

    public void tiltDown()
    {
        tilter.set(false);
    }

    public void shoot()
    {
        shooter.set(Value.kForward);
    }

    public void rearmShooter()
    {
        shooter.set(Value.kReverse);
    }

    public boolean hasBall()
    {
        return holdingBall;
    }

}
