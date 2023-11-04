// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;


public class sub_Arm extends SubsystemBase {
  public final DigitalInput ext_limitSwitch  = new DigitalInput(1);
  public final DigitalInput Upperext_limitSwitch = new DigitalInput(3);
  public final DigitalInput lift_limitSwitch = new DigitalInput(0);
  public final DigitalInput Upperlift_limitSwitch = new DigitalInput(2);

  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax liftM = new CANSparkMax(40,MotorType.kBrushless); //lift motor
  private final CANSparkMax left_extM = new CANSparkMax(6, MotorType.kBrushless); //Extension Master motor
  private final CANSparkMax right_extM = new CANSparkMax(62, MotorType.kBrushless); // Extension Motor

  private final CANSparkMax right_clampMotor = new CANSparkMax(8, MotorType.kBrushless); //clamp
  private final CANSparkMax left_clampMotor = new CANSparkMax(7, MotorType.kBrushless); //clamp

  
  private final RelativeEncoder extE = left_extM.getEncoder(); // extender motor encoder
  private final RelativeEncoder liftE = liftM.getEncoder(); // lift motor encoder
  public final DoubleSolenoid m_clampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  public boolean isClampOpen = true;

  //servo 
  
  public sub_Arm() {
    left_extM.restoreFactoryDefaults();
    right_extM.restoreFactoryDefaults();
    liftM.restoreFactoryDefaults();
    left_clampMotor.restoreFactoryDefaults();
    right_clampMotor.restoreFactoryDefaults();

    // ID 62 motor has to be inverted
    right_extM.setInverted(true);

    extE.setPosition(0.0);
    left_extM.setIdleMode(IdleMode.kBrake);
    right_extM.setIdleMode(IdleMode.kBrake);
    left_extM.setClosedLoopRampRate(1.0);
    right_extM.setClosedLoopRampRate(1.0);

    right_clampMotor.setIdleMode(IdleMode.kBrake);
    left_clampMotor.setIdleMode(IdleMode.kBrake);
    

    //servo
    //extSevro.set(0.9);  

    // Setting Current Limit
    left_extM.setSmartCurrentLimit(20);
    right_extM.setSmartCurrentLimit(20);
    liftM.setSmartCurrentLimit(35);

    right_clampMotor.setSmartCurrentLimit(20);
    left_clampMotor.setSmartCurrentLimit(20);

  }


  


  /**
   * Example command factory method.
   *
   * @return a command
   
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here 
        });
  }
  */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lift Arm Encoder: ", getliftEnc());
    SmartDashboard.putNumber("Extend Encoder: ", RobotContainer.RoundTo(extE.getPosition(), 3));
    SmartDashboard.putBoolean("Lower Arm Limit Switch: ", lift_limitSwitch.get());
    SmartDashboard.putBoolean("Lower Ext Limit Switch", ext_limitSwitch.get());
    SmartDashboard.putBoolean("isClampOpen: ", isClampOpen);
    SmartDashboard.putBoolean("Upper Ext Limit Switch: ", Upperext_limitSwitch.get());
    SmartDashboard.putBoolean("Upper Arm Limit Switch: ", Upperlift_limitSwitch.get());
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }






 public void liftArm(double speed) {
    if (lift_limitSwitch.get()){
      if (speed > 0) {
        liftM.set(0);
      } else {
        liftM.set(speed);
      }
      liftE.setPosition(0);
    } else if(Upperlift_limitSwitch.get()){
      if (speed < 0){
        liftM.set(0);
      }else{        
        liftM.set(speed);}
    }else{
      liftM.set(speed);}
  }
  




  public void extendMotor(double speed) {
    if (ext_limitSwitch.get()) {
      //if extend arm is moving outwards / if the 0 > speed
      if (speed > 0){
        left_extM.set(0);
        right_extM.set(0);
      } else {
        left_extM.set(speed);
        right_extM.set(speed);
      }
      extE.setPosition(0);
    } else if (Upperext_limitSwitch.get()){  // Upper Limit Switch
      if (speed < 0){
        left_extM.set(0.0);
        right_extM.set(0.0);
      } else {
        left_extM.set(speed);
        right_extM.set(speed);
      }
    }else{  
      left_extM.set(speed);
      right_extM.set(speed);
      }
    }

  //}


  public double getliftEnc() {
    return liftE.getPosition();
  }

  public double getExtEnc (){
    return extE.getPosition();
  }


  public void setliftEncZero() {
    liftE.setPosition(0);
  }

  //public void setextEncZero() {
  //  extE.setPosition(0);
  //}

   
  public void spinClamp(double speed) {
    right_clampMotor.set(-speed);
    left_clampMotor.set(speed);
  }


/*
  public void resetextEncoder(){
    if (ext_limitSwitch.get()){
      setextEncZero();
    }
    }*/
  



  
  public void CloseClamp(){
    if(clampState() == true){
      m_clampSolenoid.set(DoubleSolenoid.Value.kForward);
      isClampOpen = false;
      SmartDashboard.putBoolean("clamp is: ", isClampOpen);
    }
    }
  
  public void OpenClamp(){
      if (clampState()== false){
      m_clampSolenoid.set(DoubleSolenoid.Value.kReverse);
      isClampOpen = true;
      SmartDashboard.putBoolean("clamp is: ", isClampOpen);
  }

    }

    public void Open(){
      m_clampSolenoid.set(DoubleSolenoid.Value.kReverse);
      right_clampMotor.set(-0.05);
      left_clampMotor.set(0.05);
    }

    public void Close(){
      m_clampSolenoid.set(DoubleSolenoid.Value.kForward);
    }
      
    public boolean clampState(){
      return isClampOpen;
    }
     
    public void ClampSolenoid(){
      if(clampState() == true){
        CloseClamp();
      }
      else if(clampState() == false){
        OpenClamp();
      }
  
    }

}
