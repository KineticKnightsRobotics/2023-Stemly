// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class 
RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final static sub_Drive m_drive = new sub_Drive();
  private final sub_LimeLight m_limelight = new sub_LimeLight();
  private final static sub_Arm m_armSubsystem = new sub_Arm();
  private final static ExampleSubsystem m_biggy = new ExampleSubsystem();
  private static double deadzone;
  // init driver controller
  public static Joystick drvstick = new Joystick(Constants.driverstick);
  // driver buttons 
  public JoystickButton drA = new JoystickButton(drvstick, Constants.drA);
  public JoystickButton drB = new JoystickButton(drvstick, Constants.drB);
  public JoystickButton drX = new JoystickButton(drvstick, Constants.drX);
  public JoystickButton drY = new JoystickButton(drvstick, Constants.drY);
  //public JoystickButton drLB = new JoystickButton(drvstick, Constants.drLB);
  public JoystickButton drRB = new JoystickButton(drvstick, Constants.drRB);
  

  //init operator controller
  public static Joystick opstick = new Joystick(Constants.operatorstick);
  //operator buttons 
  public JoystickButton op1 = new JoystickButton(opstick, Constants.op1);
  public JoystickButton op2 = new JoystickButton(opstick, Constants.op2);
  public JoystickButton op3 = new JoystickButton(opstick, Constants.op3);
  public JoystickButton op4 = new JoystickButton(opstick, Constants.op4);
  public JoystickButton op5 = new JoystickButton(opstick, Constants.op5);
  public JoystickButton op6 = new JoystickButton(opstick, Constants.op6);
  public JoystickButton op7 = new JoystickButton(opstick, Constants.op7);
  public JoystickButton op8 = new JoystickButton(opstick, Constants.op8);
  public JoystickButton op9 = new JoystickButton(opstick, Constants.op9);
  public JoystickButton op10= new JoystickButton(opstick, Constants.op10);
  public JoystickButton op11= new JoystickButton(opstick, Constants.op11);
  public JoystickButton op12= new JoystickButton(opstick, Constants.op12);
  public JoystickButton op13= new JoystickButton(opstick, Constants.op13);
  public JoystickButton op14= new JoystickButton(opstick, Constants.op14);
  public JoystickButton op15= new JoystickButton(opstick, Constants.op15);
  public JoystickButton op16= new JoystickButton(opstick, Constants.op16);
  public JoystickButton op17= new JoystickButton(opstick, Constants.op17);
  public JoystickButton op18= new JoystickButton(opstick, Constants.op18);
  public JoystickButton op19= new JoystickButton(opstick, Constants.op19);
  public JoystickButton op20= new JoystickButton(opstick, Constants.op20);
  public JoystickButton op21= new JoystickButton(opstick, Constants.op21);
  public JoystickButton op22= new JoystickButton(opstick, Constants.op22);
  public JoystickButton op23= new JoystickButton(opstick, Constants.op23);
  

  public static Joystick catStick = new Joystick(Constants.CatStick);
  public static JoystickButton cat2 = new JoystickButton(catStick, Constants.cat2);
  public static JoystickButton cat3 = new JoystickButton(catStick, Constants.cat3);
  public static JoystickButton cat4 = new JoystickButton(catStick, Constants.cat4);



  //autonomous variables 
  public final static SendableChooser<Command> m_chooser = new SendableChooser<>();
  private String m_autoselected;
  private static final String highconebalance = "High Cone, Balance";
  private static final String lowconebalance  = "Low Cone, Balance";
  private static final String highcubebalance = "High Cube, Balance";
  private static final String lowcubebalance  = "Low Cube, Balance";
  private static final String highcone = "High Cone";
  private static final String lowcone  = "Low Cone";
  private static final String highcube = "High Cube";
  private static final String lowcube  = "Low Cube";
  private static final String park = "PARK OUTSIDE COMMUNITY";
  private static final String balance = "Balance";



  private final Command c_highcone = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Extension.high, 120),
    new sleep(m_biggy, 10),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 30),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new arm_AutoArmDown(m_armSubsystem, 0, 0, 50)  
  );

  private final Command c_lowcone = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.low, Scoring.Cones.Extension.low, 100),
    new sleep(m_biggy, 10),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 30),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new arm_AutoArmDown(m_armSubsystem, 0, 0, 50)
  );  
  private final Command c_highcube = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cubes.Lift.high, Scoring.Cubes.Extension.high, 155),
    new sleep(m_biggy, 20),
    new arm_AutoClampEject(m_armSubsystem, -0.5, 50),
    new drv_DriveDistance(m_drive, -0.3, 24),
    new arm_AutoArmDown(m_armSubsystem, 0, 0, 50),
    new sleep(m_biggy, 20),
    new drv_DriveDistance(m_drive, Constants.community_distance, -0.5)
  );  
  private final Command c_lowcube = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cubes.Lift.low, Scoring.Cubes.Extension.low, 155),
    new sleep(m_biggy, 20),
    new arm_AutoClampEject(m_armSubsystem, -0.5, 50),
    new ParallelCommandGroup(
      new drv_DriveDistance(m_drive, -0.3, 24),
      new arm_AutoArmDown(m_armSubsystem, 0, 0, 50)
    ),
    new sleep(m_biggy, 20),
    new drv_GyroTurnNumDegrees(m_drive, 180)
  ); 
  private final Command c_highconebalance = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Extension.high, 125),
    new sleep(m_biggy, 1),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 20),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new ParallelCommandGroup(
      new arm_AutoArmDown(m_armSubsystem, 0, 0, 50),
      new drv_DriveDistance(m_drive, -0.3, 5.0)
    ),
    new sleep(m_biggy, 1),
    new drv_GyroTurnNumDegrees(m_drive, 180),
    //balance 
    //new sleep(m_biggy, 100),
    new drv_GyroZerosAxis(m_drive),
    new sleep(m_biggy, 20),
    new drv_DriveDistance(m_drive, 0.3, 66),
    new drv_GyroBalance(m_drive)
  );
  public final static Command c_lowconebalance = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.low, Scoring.Cones.Extension.low, 100),
    new sleep(m_biggy, 1),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 30),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new ParallelCommandGroup(
      new arm_AutoArmDown(m_armSubsystem, 0, 0, 50),
      new drv_DriveDistance(m_drive, -0.3, 5.0)
    ),
    new sleep(m_biggy, 1),
    new drv_GyroTurnNumDegrees(m_drive, 180),
    //balance 
    //new sleep(m_biggy, 100),
    new drv_GyroZerosAxis(m_drive),
    new sleep(m_biggy, 20),
    new drv_DriveDistance(m_drive, 0.3,66),
    new drv_GyroBalance(m_drive)
  );

  private final Command c_balance = new SequentialCommandGroup(
    new sleep(m_biggy, 20),
    new drv_DriveDistance(m_drive, -0.3, 9),
    new sleep(m_biggy, 20),
    new drv_GyroTurnNumDegrees(m_drive, 180),
    //balance 
    //new sleep(m_biggy, 100),
    new drv_GyroZerosAxis(m_drive),
    new sleep(m_biggy, 20),
    new drv_DriveDistance(m_drive, 0.26, 63.5),
    new drv_GyroBalance(m_drive)
  );
  private final Command c_park = new SequentialCommandGroup(
    new drv_DriveDistance(m_drive, -0.30, 60)
  );





  private final Command c_highcone_park = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high-1, Scoring.Cones.Extension.high-1, 120),
    new sleep(m_biggy, 1),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 20),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new ParallelCommandGroup(
      new arm_AutoArmDown(m_armSubsystem, 0, 0, 10),
      new drv_DriveDistance(m_drive, -0.3, Constants.community_distance)
    )
  );
  private final Command c_lowcone_park = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.low, Scoring.Cones.Extension.low, 100),
    new sleep(m_biggy, 1),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 20),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new ParallelCommandGroup(
      new arm_AutoArmDown(m_armSubsystem, 0, 0, 10),
      new drv_DriveDistance(m_drive, -0.3, Constants.community_distance)
    )
  );

  private final Command c_highcone_conepickup = new SequentialCommandGroup(
    /*
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Extension.high, 125),
    new sleep(m_biggy, 1),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 20),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new arm_AutoArmDown(m_armSubsystem, 0, 0, 50),
    new sleep(m_biggy, 10),
    new drv_DriveDistance(m_drive, -0.2, 10),
    new drv_GyroTurnNumDegrees(m_drive, 180),
    new sleep(m_biggy, 10),
     */
    new ll_ChangePipeline(m_limelight, 0),
    new sleep(m_biggy, 10),
    new arm_AutoArmUp(m_armSubsystem, Constants.ll_scan.Lift, Constants.ll_scan.Extension, 100),
    new sleep(m_biggy, 10),
    new ll_TargetSteerDriveDist(m_limelight, m_drive, 0.2, 20),
    new sleep(m_biggy, 10),
    new arm_AutoArmUp(m_armSubsystem, Constants.Intake.Lift, Constants.Intake.Extension, 20),
    new sleep(m_biggy, 10),
  
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 1),
    //new arm_clampWheelSpin(m_armSubsystem, 0.1),
    new ParallelCommandGroup(
      new arm_clampWheelSpin(m_armSubsystem, 0.05, -1),
      new drv_DriveDistance(m_drive, 0.2, 40)
    ),
    new sleep(m_biggy, 10),
    new drv_DriveDistance(m_drive, -0.2, 40),
    new sleep(m_biggy, 10),
    new drv_GyroTurnNumDegrees(m_drive, -180),
    new drv_DriveDistance(m_drive, 0.2, 20)
    /*

    new drv_DriveDistance(m_drive, 0.2, 10),
    new sleep(m_biggy, 10),
    new ll_ChangePipeline(m_limelight, 3),
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Lift.high, 120),
    new sleep(m_biggy, 10),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 30),
    new arm_clampChangeSolenoid(m_armSubsystem)
   */
    );




  private final Command c_highcone_pickup = new SequentialCommandGroup(
    new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Extension.high, 120),
    new sleep(m_biggy, 10),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new sleep(m_biggy, 30),
    new arm_clampChangeSolenoid(m_armSubsystem),
    new arm_AutoArmDown(m_armSubsystem, 0, 0, 50),
    new sleep(m_biggy, 10),
    new drv_DriveDistance(m_drive, -0.2, 198.75/3),//drive outside community
    new drv_GyroTurnNumDegrees(m_drive, 180),
    new sleep(m_biggy, 10),
    new arm_AutoArmUp(m_armSubsystem, Constants.ll_scan.Lift, Constants.ll_scan.Extension, 120),
    new ll_ChangePipeline(m_limelight, 0),
    new ll_TargetSteerDriveDist(m_limelight, m_drive, 0.2, 5/3),
    new arm_AutoArmUp(m_armSubsystem, Constants.Intake.Lift, Constants.Intake.Extension, 100),
    new drv_DriveDistance(m_drive, 0.2, 5/3),
    new drv_DriveDistance(m_drive, -0.2, 10/3),
    new sleep(m_biggy, 10),
    new drv_GyroTurnNumDegrees(m_drive, -180),
    new sleep(m_biggy, 10),
    new drv_DriveDistance(m_drive, -0.2, 198.75/3)
  );
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // sets default commands for each subsystem 
    m_armSubsystem.setDefaultCommand(new arm_ManualControl(m_armSubsystem));


 

    m_drive.setDefaultCommand(new drv_Joystickdrive(m_drive));
    // Configure the trigger bindings
    configureBindings();

    //m_chooser.setDefaultOption(highcone, c_highcone);
    m_chooser.setDefaultOption(highconebalance, c_highconebalance);
    //m_chooser.setDefaultOption("highcone_conepickup", c_highcone_conepickup);
    //m_chooser.setDefaultOption("HighCone", c_highcone);
    //m_chooser.setDefaultOption("DEFAULT, Low cone balance", c_lowconebalance);

    //m_chooser.addOption(highconebalance, c_highconebalance);
    m_chooser.addOption(lowconebalance, c_lowconebalance);
    //m_chooser.addOption(lowconebalance,);
   // m_chooser.addOption(highcubebalance,);
    //m_chooser.addOption(lowcubebalance,);

    //m_chooser.addOption(highcone, c_highcone);
    //m_chooser.addOption(lowcone,c_lowcone);
    m_chooser.addOption(lowcone, c_lowcone);
    //m_chooser.addOption("HighCone, Cube Pick Up", c_highcone_pickup);
    //m_chooser.addOption(lowcube,c_lowcube);
    m_chooser.addOption(highcone, c_highcone);
    m_chooser.addOption("HighCone, Park", c_highcone_park);
    m_chooser.addOption("LowCone, Park", c_lowcone_park);
    
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //drA.whileTrue(new ll_TargetSteer(m_limelight,m_drive));
    
    //drX.whileTrue(new drv_GyroTurnNumDegrees(m_drive, 180));
    drA.whileTrue(new arm_AutoArmDown(m_armSubsystem, -1, 0, 30));
    drX.whileTrue(new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.low, Scoring.Cones.Extension.low-20, 100));
    drY.whileTrue(new arm_clampChangeSolenoid(m_armSubsystem));
    //drY.whileTrue(new drv_GyroBalance(m_drive));
    //drX.whileTrue(new drv_newBalance(m_drive));
    //drX.whileTrue(new drv_balancEnc(m_drive, 38));
    drB.whileTrue(new arm_AutoArmUp(m_armSubsystem, Scoring.Cones.Lift.high, Scoring.Cones.Extension.high, 120));
    drRB.whileTrue (new SequentialCommandGroup(
      new arm_AutoArmUp(m_armSubsystem, Intake.Lift, Intake.Extension, 30),
      new sleep(m_biggy, 20),
      new arm_clampSet(m_armSubsystem, true)
    ));

    //change limelight pipelines
    op4.onTrue(new ll_ChangePipeline(m_limelight,0));// cube
    op5.onTrue(new ll_ChangePipeline(m_limelight,1));// cone
    op11.onTrue(new SelectAuto("lowcone_balance"));
    op12.onTrue(new SelectAuto("lowcone"));
    op19.whileTrue(new ll_TargetSteer(m_limelight, m_drive));

    //op9.onTrue(new ll_ChangePipeline(m_limelight,2));
    op20.whileTrue(new drv_GyroZerosAxis(m_drive));

    op22.onTrue(new ll_ChangePipeline(m_limelight, 2));//tapelow
    op23.onTrue(new ll_ChangePipeline(m_limelight, 3));//tapehigh
    
    op10.onTrue(new ll_ChangePipeline(m_limelight,4));//apriltag
    //op20.whileTrue(new drv_GyroZerosAxis(m_drive));

    

    


    //op11.whileTrue(new SequentialCommandGroup()
    //op11.whileTrue(new drv_DriveDistance(m_drive, 0.2, 24));
    //op13.whileTrue(new drv_DriveDistance(m_drive, -0.2, 24));

    //spins the wheels of the clamp
 
    //open and closes clamp
    cat2.whileTrue(new arm_clampChangeSolenoid(m_armSubsystem));
    //cat3.whileTrue(new ll_TargetSteerDriveDist(m_limelight, m_drive, 0.2, 5));
    //cat2.whileTrue(new arm_clampChangeSolenoid(m_armSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return m_ExampleCommand;

    

    return m_chooser.getSelected();
  }
   

  public static double DriveY (){
    double y =0.0;
    if (drvstick.getRawAxis(Constants.driverforward) > 0.1 || drvstick.getRawAxis(Constants.driverforward) < -0.1) {y = drvstick.getRawAxis(Constants.driverforward);}
    return y;
  }

  public static double DriveX(){
    double x=0.0;
    if (drvstick.getRawAxis(Constants.driversteer) > 0.1 || drvstick.getRawAxis(Constants.driversteer) < -0.1) {x = drvstick.getRawAxis(Constants.driversteer);}
    return x;
  }

  // when the opstick is shifted in the y axis, the value is returned to the command using this function
  public static double ArmY () {
    return opstick.getRawAxis(1);
  }

  

  public static double CatArmY(){
    deadzone = catStick.getRawAxis(1);
  
    if (deadzone < -0.10|| deadzone > 0.10){
      return deadzone;
    } else {
      return 0;
    }
    
  }

  
  public static double CatArmX(){
    
    deadzone = catStick.getRawAxis(0);
    if(deadzone > 0.10 || deadzone < -0.10){
      return deadzone;
    } else {
      return 0;
    }
    
    //return catStick.getRawAxis(0);

  }


  public static int clampSpeed() {
    if (cat3.getAsBoolean()) {

      return -1;

    } else if (cat4.getAsBoolean()) {

      return 1;

    } else {

      return 0;

    }
    
  
  }
  
  public static double CatArmRotate(){
    return catStick.getRawAxis(3);
  }
  public static boolean drRT(){
    return drvstick.getRawAxis(3) > 0.75;
  }
  public static boolean drLT(){
    return drvstick.getRawAxis(2) > 0.75;
  }

  public static boolean drLB() {
    return drvstick.getRawButton(Constants.drLB);
  }



  public static double RoundTo (double num, int decimal_place){
    return (Math.round(num * (10 * decimal_place))/(10 * decimal_place));
  }

}
