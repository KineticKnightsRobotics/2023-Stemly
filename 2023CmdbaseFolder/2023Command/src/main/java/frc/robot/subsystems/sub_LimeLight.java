// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sub_LimeLight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("/limelight");  //create network table instance
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");  //create entries
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  public final double highpost= 0;
  public final double medpost = 0;
  double heightofLimelight = 21; // in inches
    double heightofTopPole = 45.69; // in inches
    double height = heightofTopPole - heightofLimelight;


  public sub_LimeLight() {}

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
    SmartDashboard.putBoolean("tv: ", ll_tv());
    SmartDashboard.putNumber("tx: ", tx.getDouble(0));
    SmartDashboard.putNumber("distance in inches", returnDistance());
    SmartDashboard.putBoolean("Object Within Range: ", TargetWithinRange());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public double ll_tx() {
    return tx.getDouble(0.0); //returns value of TX, inside brackets is defualt value
  }

  public double ll_ty (){
    return ty.getDouble(0.0);
  }

  public double ll_ta (){
    return ta.getDouble(0.0);
  }

  public boolean ll_tv (){
    double target = tv.getDouble(0);
    return target > 0;
  }




  public double returnDistance(){
    double y = ll_ty();

    if(y < 0){// if target is below the reticle
      double yAngle = Math.toRadians(y + 90);
      double distance = height / (Math.tan(yAngle));
      return distance;
    } else if (y > 0){
      double yAngle = Math.toRadians(y);
      double distance = height * (Math.tan(yAngle));
      return distance;
    }
    else {
      return 0;
    }


}

public boolean TargetWithinRange (){
  double ll_tx = tx.getDouble(0.0);
  return ll_tx < 1 && ll_tx > -1;
}



  public void setPipline (double id){  // Changes the limelight pipline to specified id
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(id);
    //pipelineEntry.setNumber(id);
  }
}
