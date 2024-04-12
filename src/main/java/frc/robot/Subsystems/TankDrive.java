package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Utility.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * This will cover the basics of a Differential (tank-drive) system that is compatible with Path Planner
 * This will also be used for examples of subclassing commands
 */
public class TankDrive extends SubsystemBase
{
    //Declare global variables here. Initialize them in constructor
    CANSparkMax frontLeft;
    CANSparkMax backLeft;
    CANSparkMax frontRight;
    CANSparkMax backRight;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    AHRS navX;

    DifferentialDriveKinematics m_kinematics;
    DifferentialDriveOdometry m_odometry;

    //Initialize globals, set basic configuration (invert 1 side, get encoders from motors, set encoder conversions).
    public TankDrive()
    {
        frontLeft = new CANSparkMax(kFrontLeftCANID, MotorType.kBrushless);
        backLeft = new CANSparkMax(kBackLeftCANID, MotorType.kBrushless);
        frontRight = new CANSparkMax(kFrontRightCANID, MotorType.kBrushless);
        backRight = new CANSparkMax(kBackRightCANID, MotorType.kBrushless);

        leftEncoder = frontLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        leftEncoder.setPositionConversionFactor(kMetersPerRevolution);
        rightEncoder.setPositionConversionFactor(kMetersPerRevolution);
        leftEncoder.setVelocityConversionFactor(kMPS_Per_RPM);
        rightEncoder.setVelocityConversionFactor(kMPS_Per_RPM);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontLeft.setInverted(true);

        navX = new AHRS();

        m_kinematics = new DifferentialDriveKinematics(kWheelWidth);
        m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

        /* AutoBuilder is what pathplanner uses to run. Each function gives it a way to interact with our drive
        * poseSupplier tells it where we are
        * poseResetter lets it tell us the starting position for the auto
        * speedSupplier tells it how fast we are going
        * driveMotorsWithChassisSpeeds gives it a way to power the motors
        * areWeRed tells it if we should flip the path
        */
        AutoBuilder.configureLTV
        (
            poseSupplier,
            poseResetter,
            speedSupplier,
            driveMotorsWithChassisSpeeds,
            0.02,
            new ReplanningConfig(),
            areWeRed,
            this
        );
    }

    /*
     * Periodic runs constantly every command loop (20ms). Anything you want to always do can be done here. The most important thing is updating odometry
     */
    @Override
    public void periodic() 
    {
        m_odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());   
    }

    /*
     * Below are function patterns called producers and consumers. If you have heard of getters and setters from a class, these are very similar
     * A producer is a function that takes no parameters, and returns the given type. 
     * Supplier<Double> takes nothing and returns a double. Supplier<Double> a = () -> return 3.0; is the same as double a() { return 3.0; }
     * A consumer is a function that takes one parameter of the given type, and returns nothing.
     * Consumer<Double> b = (num) -> counter = counter + num; is the same as void b(double num) {counter = counter + num; }
     * 
     * If the supplier/consumer and the basic method are the same, why bother with the supplier/consumer?
     * The answer is that a supplier or consumer can be treated like a variable and be passed into a method.
     * This is very useful for keeping things separated in scope. For example, I might want to send a joystick axis to a command, but keep the joystick in RobotContainer
     * I can make a supplier for the joystick axis, and then send it into my command constructor.
     */
    Supplier<Pose2d> poseSupplier = () -> m_odometry.getPoseMeters();

    Consumer<Pose2d> poseResetter = (pose) -> m_odometry.resetPosition(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);

    Supplier<ChassisSpeeds> speedSupplier = () -> m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()));

    /*
     * This is where the magic happens. Pathplanner gives us the chassis speeds it wants, and we command the drive motors accordingly
     * For teleop you can reuse this function - generate and send a chassisSpeeds object based on joysticks
     * 
     * This function is currently as simple as possible. There is more to do before pathplanner will be able to follow accurately.
     * Step 0: Verify odometry
     * Step 1: Add PID
     * Step 2: Add Feedforward
     * Step 3: Add acceleration to feedforward
     */
    Consumer<ChassisSpeeds> driveMotorsWithChassisSpeeds = (chassisSpeeds) ->
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

        double leftSpeedTarget = wheelSpeeds.leftMetersPerSecond;
        double rightSpeedTarget = wheelSpeeds.rightMetersPerSecond;

        frontLeft.set(leftSpeedTarget / maxSpeed);
        frontRight.set(rightSpeedTarget / maxSpeed);
    };

    /*
     * This feels unnecesarily complicated, but it is because we can sometimes not have any alliance before connecting to the field.
     */
    BooleanSupplier areWeRed = () -> 
    { 
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    };

    
            
}
