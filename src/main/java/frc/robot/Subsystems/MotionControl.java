package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotionControl extends SubsystemBase
{
    CANSparkMax armMotor;
    DutyCycleEncoder armPosition;

    ProfiledPIDController armPID;
    ArmFeedforward armFF;

    double Kp = 1; //Position constant - based on distance from setpoint
    double Ki = 0; //Integral constant - corrects steady state error - usually uncessary
    double Kd = 0; //Derivative constant - based on velocity towards setpoint - CAREFUL - AMPLIFIES SENSOR NOISE
    double maxVelocity = 1; //Max DESIRED (not possible) velocity. For arm, radians per second
    double maxAccel = 1; //Max DESIRED (not possible) acceleration. For arm, radians per second squared

    double Kg = 0.33; //Volts to counteract gravity. For elevator, always same volts. For arm, this is internally multiplied by cos(theta)
    double Ks = 0.05; //Volts to counteract friction
    double Kv = 1.6; //Volts to achieve a velocity
    double Ka = 0.01; //Volts to achieve an acceleration - probably very small and can usually be skipped for a position controller

    double currentAngleRadians;
    /*
     * I HIGHLY recommend reading all of the articles at https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
     * It covers PID, Feedforward, PID+FF together, motion profiles, and combining them all together
     * In short:
     * PID corrects for errors in the position
     * Feedforward uses mechanism characterization to guess how to move
     * Motion profiling prevents sudden jumps
     */
    public MotionControl()
    {
        armMotor = new CANSparkMax(1, MotorType.kBrushless);

        armPosition = new DutyCycleEncoder(0);

        armPID = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxVelocity, maxAccel));
        armPID.setTolerance(Units.degreesToRadians(2));

        armFF = new ArmFeedforward(Ks, Kg, Kv, Ka);
    }

    @Override
    public void periodic() 
    {
        //use an offset to make this so that 0 degrees is horizontal and 90 degrees is vertical
        currentAngleRadians = 2*Math.PI * armPosition.getAbsolutePosition();
    }

    public boolean moveToLoadPosition()
    {
        return moveToPosition(Units.degreesToRadians(30));
    }

    public boolean moveToShootPosition()
    {
        return moveToPosition(Units.degreesToRadians(120));
    }

    double lastSpeed;
    private boolean moveToPosition(double angleSPRadians)
    {
        double PIDoutput = armPID.calculate(currentAngleRadians, angleSPRadians);

        double acceleration = armPID.getSetpoint().velocity - lastSpeed / 0.02;

        double FFoutput = armFF.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity, acceleration);

        armMotor.setVoltage(PIDoutput + FFoutput);

        lastSpeed = armPID.getSetpoint().velocity;
        return armPID.atGoal();
    }
}
