package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorsSubsystem extends SubsystemBase
{
    //NavX gyro to read roll, pitch, and yaw
    AHRS navX;

    //Absolute encoder - remembers its position between power off
    //Uses the white pin from a Rev Thru Bore to a DIO
    //Reads 0 to 1 and wraps after 1 rotation.
    DutyCycleEncoder revThruBoreAbsolute;

    //Relative or Quadrature encoder - forgets its position
    //You can use the rev thru bore with blue and yellow pins to 2 different DIO ports
    //Many other encoders also work this way
    Encoder RioQuadratureEncoder;

    //Encoder built into Neo
    CANSparkMax neo;
    RelativeEncoder neoEncoder;

    //Simple digital input such as beam break or limit switch
    DigitalInput limitSwitch;
    
    //Analog sensors are rare, and you have to be careful with noise. A common use is an analog IR distance sensor
    AnalogInput distanceSensor;

    //Rev color sensor. Uses I2C, but NEVER use the main I2C on the rio, always use the MXP expansion I2C.
    ColorSensorV3 colorSense;

    //The roborio has a built in accelerometer that gives you your acceleration in X, Y, and Z. It isn't nearly accurate enough for odometry
    //Some teams used it for auto balance. You can infer your pitch based on how much of gravity is in the Z axis.
    BuiltInAccelerometer accelerometer;

    public SensorsSubsystem()
    {
        navX = new AHRS();

        revThruBoreAbsolute = new DutyCycleEncoder(0);

        RioQuadratureEncoder = new Encoder(1, 2);

        neo = new CANSparkMax(1, MotorType.kBrushless);
        neoEncoder = neo.getEncoder();

        limitSwitch = new DigitalInput(3);

        distanceSensor = new AnalogInput(0);

        colorSense = new ColorSensorV3(edu.wpi.first.wpilibj.I2C.Port.kMXP);

        accelerometer = new BuiltInAccelerometer();       
        
    }   
    
    @Override
    public void periodic() 
    {
        navX.getRate();
        navX.getRotation2d();
        navX.getYaw();
        navX.getPitch();
        navX.getRoll();

        revThruBoreAbsolute.setPositionOffset(0);
        revThruBoreAbsolute.getAbsolutePosition();

        RioQuadratureEncoder.setDistancePerPulse(1);
        RioQuadratureEncoder.getDistance();
        RioQuadratureEncoder.getRate();

        neoEncoder.setPositionConversionFactor(1);
        neoEncoder.setVelocityConversionFactor(60);
        neoEncoder.getPosition();
        neoEncoder.getVelocity();

        limitSwitch.get();

        distanceSensor.getVoltage();

        colorSense.getRed();
        colorSense.getBlue();
        colorSense.getGreen();
        colorSense.getColor();
        colorSense.getProximity();
        
        accelerometer.getX();
        accelerometer.getY();
        accelerometer.getZ();
    }
}
