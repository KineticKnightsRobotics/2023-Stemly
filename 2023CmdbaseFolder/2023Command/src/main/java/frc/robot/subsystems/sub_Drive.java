package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class sub_Drive extends SubsystemBase{
    private SlewRateLimiter filter = new SlewRateLimiter(0.5);

    private static int leftlead  = 3;
    private static int rightlead = 1;
                                                //ids for spark maxes
    private static int leftfol = 4;
    private static int rightfol= 2;

    // Gyroscope Initialization
    public final static AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Calibration Values
    public static double calibrationPitch = gyro.getRoll();
    public static double calibrationRoll = gyro.getPitch();
    public static double calibrationYaw = gyro.getYaw(); 

    //speed multipliers
    public static double maxspeed= 0.5; //maximum speed in %tage
    public static double maxsteer= 0.6; //maximum steer in %tage
    public static double shifterspeed;


    public CANSparkMax m_leftMotor = new CANSparkMax(leftlead, MotorType.kBrushless);
    public CANSparkMax m_rightMotor = new CANSparkMax(rightlead, MotorType.kBrushless);
    private CANSparkMax f_leftMotor = new CANSparkMax(leftfol, MotorType.kBrushless);
    private CANSparkMax f_rightMotor = new CANSparkMax(rightfol, MotorType.kBrushless);

    // Initializing Encoders

    public RelativeEncoder leftMotorEncoder = m_leftMotor.getEncoder();
    // Compressor
    private RelativeEncoder rightMotorEncoder = m_rightMotor.getEncoder();
    private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    public static boolean isLowGear = false;

    //timer for velocity
    public static double velTimer = 0;
      

    /** Creates a new ExampleSubsystem. */
  public sub_Drive() {
    //Compressor compressor = new Compressor()
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    f_leftMotor.restoreFactoryDefaults();
    f_rightMotor.restoreFactoryDefaults();

    m_leftMotor.setClosedLoopRampRate(1.0);
    m_rightMotor.setClosedLoopRampRate(1.0);
    f_leftMotor.setClosedLoopRampRate(1.0);
    f_rightMotor.setClosedLoopRampRate(1.0);
    // Setting Encoder Positions to 0
    leftMotorEncoder.setPosition(0.0);
    rightMotorEncoder.setPosition(0.0);

        // setting the right motor inverte
    m_rightMotor.setInverted(true);
    f_rightMotor.setInverted(true);
    //m_leftMotor.setInverted(true);
    //f_leftMotor.setInverted(true);

    // Setting Follower Motor to follow Master Motor
    f_leftMotor.follow(m_leftMotor);
    f_rightMotor.follow(m_rightMotor);

    m_leftMotor.setIdleMode(IdleMode.kBrake);
    f_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    f_rightMotor.setIdleMode(IdleMode.kBrake);


    m_leftMotor.setSmartCurrentLimit(35);
    f_leftMotor.setSmartCurrentLimit(35);
    m_rightMotor.setSmartCurrentLimit(35);
    f_rightMotor.setSmartCurrentLimit(35);

    // ReCalibrates the Gyro
    resetGyro();

    // Disables the compressor
    //compressor.disable();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch: ", ObtainPitch());
    SmartDashboard.putNumber("Yaw: ", ObtainYaw());
    SmartDashboard.putNumber("Roll: ", ObtainRoll());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void ArcadeDrive(double speed, double steer) {
    // Moving Forward/Bacward

    double lSpeed = speed - steer;
    double rSpeed = speed + steer;
    m_leftMotor.set(lSpeed);
    m_rightMotor.set(rSpeed);
    SmartDashboard.putNumber("last pitch since controller input", ObtainPitch());

  }

  // Gyro User Defined Functions
  public void resetGyro (){
    gyro.calibrate();  // Calibrates the Gyro
    gyro.reset();   // Resets the Yaw to zero
  }

  // Gyro Functions: Obtaining the Axis Values (Yaw, Roll, and Pitch)

  public double ObtainPitch (){ // Gets the Pitch (Pitch and roll are swapped)
    return gyro.getRoll() - calibrationPitch;
  }

  public double ObtainYaw (){
    return gyro.getYaw() - calibrationYaw;
  }

  public double ObtainRoll (){
    return gyro.getPitch() - calibrationRoll;
  }



  // Gyro Function: Reseting Yaw to Zero
  
  public void ZeroYaw (){
    gyro.zeroYaw(); // Sets yaw to zero
    calibrationYaw = gyro.getYaw(); // Finds new calibrated Yaw
  }


  // Gyro Function: Return Boolean; telling us if the robot is on a slope
  public boolean OnSlope (){
    double p = ObtainPitch();
    return p > 1.5 || p < -1.5;  // Range where we are not on a slope are between 3 and -3
  }

  public void TurnCompressOnOrOff (){
    if (compressor.isEnabled()){
      compressor.disable();
    } else {
      compressor.enableDigital();
    }
  }

  public void ShiftLowGear (){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    
  }


  public void ShiftHighGear(){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

public void ZeroEncoders(){
  leftMotorEncoder.setPosition(0);
  rightMotorEncoder.setPosition(0);

}

public double getDriveEncoder(){
  return leftMotorEncoder.getPosition();
}





}


