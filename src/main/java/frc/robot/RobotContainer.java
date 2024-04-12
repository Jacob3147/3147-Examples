// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.InlineCommands;
import frc.robot.Subsystems.MotionControl;
import frc.robot.Subsystems.ReturnedCommands;
import frc.robot.Subsystems.SensorsSubsystem;
import frc.robot.Subsystems.TankDrive;

public class RobotContainer 
{
    CommandXboxController mainXbox = new CommandXboxController(0);
    CommandXboxController secondaryXbox = new CommandXboxController(1);

    TankDrive m_DriveSubsystem = new TankDrive();
    InlineCommands m_inline = new InlineCommands();
    SensorsSubsystem m_sensors = new SensorsSubsystem();
    MotionControl m_MotionControl = new MotionControl();
    ReturnedCommands m_returnedCommand = new ReturnedCommands();

    SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        


        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        configureBindings();
    }

    private void configureBindings() 
    {
        /*
        Commands and Triggers
        This configureBindings() is only run once, but works forever until the robot stops. How?
        This functions sets up Triggers and binds them to Commands. From then on the Command Scheduler listens for the triggers and activates commands as appropriate
        If you want to run a command independently of the command scheduler, you can use myCommand.schedule()

        The most common Triggers are the ones created by CommandXboxController
        You can simply do myController.a().whileTrue(myCommand)
        There is also whileFalse, onTrue, onFalse
        You can combine triggers using AND and OR
        myController.a().and(myController.b()).whileTrue(myCommand)

        Soemtimes you'll want to create your own trigger. You can do this from a boolean fairly easily:
        Trigger myTrigger = new Trigger(() -> myBoolean);
        myTrigger.onTrue(myCommand);
        You can save a line with (new Trigger(() -> myBoolean)).onTrue(myCommand)
        */
        
        
        /*
        InlineCommands subsystem
        You can turn any function into a Command by calling it as Commands.run(() -> function) (or runOnce)
        You can combine commands together using groups and decorators
        Commands.sequence(first, second, third, fourth...)
        Commands.parallel(a, b, c) - ends when all end
        Commands.deadline(a, b, c) - ends when A ends only
        Commands.race(a, b, c) - ends when any ends
        myCommand.andThen()
        myCommand.until()
        myCommand.raceWith()
        myCommand.alongWith()
        myCommand.repeatedly()
        Commands.waitSeconds(1)
        */
        
        //Shoot
        mainXbox.rightTrigger(0.5).onTrue
        (
            Commands.sequence
            (
                Commands.runOnce(() -> m_inline.shoot()),
                Commands.waitSeconds(1),
                Commands.runOnce(() -> m_inline.rearmShooter())

            )
        );

        //Intake
        mainXbox.leftTrigger(0.5).whileTrue
        (
            Commands.parallel
            (
                Commands.runOnce(() -> m_inline.intake()),
                Commands.runOnce(() -> m_inline.tiltDown())
            )
            .until(() -> m_inline.hasBall())
            .andThen
            (
                Commands.runOnce(() -> m_inline.stopIntake()),
                Commands.runOnce(() -> m_inline.tiltUp())
            )
        );



        /*
         * Returned commands subsystem
         * Same functionality as inline command subsystem
         */
        mainXbox.a().onTrue(m_returnedCommand.shootSequence());
        mainXbox.b().whileTrue(m_returnedCommand.intakeSequence());
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected();
    }
}
