package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This is an example of creating commands in the subsystem, using methods that return Commands
 * This is also often referred to as Command Factory structure
 */
public class ReturnedCommands extends SubsystemBase
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

    public ReturnedCommands()
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

    public Command intakeSequence()
    {
        return Commands.parallel
        (
            intake(),
            tiltDown()
        )
        .until(hasBall())
        .andThen
        (
            stopIntake(),
            tiltUp()
        );
    }

    public Command shootSequence()
    {
        return  Commands.sequence
        (
            shoot(),
            Commands.waitSeconds(1),
            rearmShooter()

        );
    }

    public Command intake()
    {
        return Commands.runOnce(() -> intake.set(1));
    }

    public Command outtake()
    {
        return Commands.runOnce(() -> intake.set(-1));
    }

    public Command stopIntake()
    {
        return Commands.runOnce(() -> intake.set(0));
    }

    public Command tiltUp()
    {
        return Commands.runOnce(() -> tilter.set(true));
    }

    public Command tiltDown()
    {
        return Commands.runOnce(() -> tilter.set(false));
    }

    public Command shoot()
    {
        return Commands.runOnce(() -> shooter.set(Value.kForward));
    }

    public Command rearmShooter()
    {
        return Commands.runOnce(() -> shooter.set(Value.kReverse));
    }

    public Trigger hasBall()
    {
        return new Trigger(() -> holdingBall);
    }

}
