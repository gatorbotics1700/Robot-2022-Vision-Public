// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Autonomous.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;

import edu.wpi.first.hal.util.CheckedAllocationException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.TransitionSubsystem.TransitionState;
import frc.robot.Autonomous.AutonomousBase.AutonomousStates;
import frc.robot.Autonomous.AutonomousBase.Paths;
import frc.robot.subsystems.ShootSubsystem.ShootState;
import frc.robot.subsystems.SensorSubsystem.SensorStates;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private static final String kDefaultAuto = "Default";
  //private static final String kCustomAuto = "My Auto";
  public Paths m_autoSelected = Paths.BOOP3;
  private final SendableChooser<Paths> autoChooser = new SendableChooser<Paths>();
  private final SendableChooser<Double> pipelineChooser = new SendableChooser<Double>();
  private static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static final AutonomousBasePID autonomousBasePID = new AutonomousBasePID();
  private static final TransitionSubsystem transitionSubsystem = new TransitionSubsystem();
  private static final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private static final SensorSubsystem sensorSubsystem = new SensorSubsystem();
  private static final Telemetry telemetry = new Telemetry();
  private static final Faults fault = new Faults();
  private static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(); 
  private static final TurretSubsystem turretSubsystem = new TurretSubsystem();

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    System.out.println("I have awoken!");
    autoChooser.addOption("BOOP1", Paths.BOOP1);
    autoChooser.addOption("BOOP3", Paths.BOOP3);
    autoChooser.addOption("SHOOTANDCROSS", Paths.SHOOTANDCROSS);
    autoChooser.addOption("ALTSHOOTTWICE", Paths.ALTSHOOTTWICE);
    autoChooser.addOption("SMIO", Paths.SMIO);
    autoChooser.addOption("BARELYCROSS", Paths.BARELYCROSS);
    autoChooser.addOption("ONLYSHOOT", Paths.ONLYSHOOT);
    autoChooser.addOption("DSHOOTTWICE", Paths.DSHOOTTWICE);
    autoChooser.addOption("DALTSHOOTTWICE", Paths.DALTSHOOTTWICE);
    autoChooser.setDefaultOption("SHOOTTWICE", Paths.SHOOTTWICE);
    autoChooser.setDefaultOption("SM", Paths.SM);
    autoChooser.setDefaultOption("SMI", Paths.SMI);
    pipelineChooser.setDefaultOption("0", 0.0);
    pipelineChooser.addOption("1", 1.0);
    SmartDashboard.putData("pipeline chooser", pipelineChooser);
    SmartDashboard.putData("Auto Paths", autoChooser);
    autonomousBasePID.init();
    transitionSubsystem.init();
    intakeSubsystem.init();
    telemetry.init(); //bringing back camera
    sensorSubsystem.setup(); 
    climberSubsystem.init();
    shootSubsystem.init();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /* commented out on 7/20 to cut down on # of tasks robot has to do
    //System.out.println("is shooting twice: " + ShootSubsystem.isTwice);
    SmartDashboard.putBoolean("is shooting twice", ShootSubsystem.isTwice); //IF CODE ERRORS, TAKE OUT THIS LINE
    SmartDashboard.putNumber("current shooter rpm", shootSubsystem.getRPM());
    SmartDashboard.putString("Intake state", IntakeSubsystem.intakeState.name());
    SmartDashboard.putString("Transition state", TransitionSubsystem.state.name());
    SmartDashboard.putString("Shooter state", ShootSubsystem.state.name());
    SmartDashboard.putString("Climber state", ClimberSubsystem.climberState.name());
    SmartDashboard.putString("Sensor state", SensorSubsystem.state.name()); 

    //SmartDashboard.putNumber("ultrasonic range", SensorSubsystem.ultrasonic.getRangeInches());  //siona wants this to output a color instead of a number - purple (within 8 in) and orange (outside 8 in)
    SmartDashboard.putBoolean("ultrasonic range", SensorSubsystem.within8Inches);
    LimelightSubsystem.limelightData();
    
    intakeSubsystem.periodic();
    transitionSubsystem.periodic();
    sensorSubsystem.periodic();

    if (fault.hasAnyFault()){
      SmartDashboard.putString("faults", fault.toString());
    }
    */
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */


  public void autonomousInit() {
    m_autoSelected = autoChooser.getSelected();
    autonomousBasePID.init();
    System.out.println("Auto selected: " + m_autoSelected);
    driveSubsystem.setCoastMode();
    //climberSubsystem.setStateClimber(ClimberStates.RETRACTING);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //m_autoSelected = Paths.SHOOTANDCROSS; //IF AUTO CHOOSER DOESNT WORK

    AutonomousBasePID.path = m_autoSelected;
    if(m_autoSelected == Paths.BOOP1){
      System.out.println("BOOP1");
    } else if(m_autoSelected == Paths.SHOOTTWICE){
      System.out.println("SHOOTTWICE");
      autonomousBasePID.periodic(-14.75, -132.2, 37.579, 0, -37.579, 142.2, 9.75); //increase d4 from 7.75 -> 9.75, monterey was 8.75
    } else if (m_autoSelected == Paths.SHOOTANDCROSS){
      autonomousBasePID.periodic(-29.5, 0, 0, 0, 0, 0, 0);
    } else if (m_autoSelected == Paths.ALTSHOOTTWICE){
      autonomousBasePID.periodic(0, 146, 48, 0, -38, -153, 0); 
    } else if (m_autoSelected == Paths.SM){
      autonomousBasePID.periodic(-43, 0, 0, 0, 0, 0, 0);
    } else if (m_autoSelected == Paths.SMI){
      autonomousBasePID.periodic(0, 160, 55, 0, 0, 0, 0);
    } else if (m_autoSelected == Paths.SMIO){
      autonomousBasePID.periodic(0, 150, 55, 30, 0, 0, 0); 
    } else if (m_autoSelected == Paths.ONLYSHOOT){
      autonomousBasePID.periodic(0, 0, 0, 0, 0, 0, 0);
    } else if (m_autoSelected == Paths.BARELYCROSS){
      autonomousBasePID.periodic(0, -30, 0, 0, 0, 0, 0);
    }else if(m_autoSelected == Paths.DSHOOTTWICE){
      autonomousBasePID.periodic(-14.75, -132.2, 37.579, 0, 0, 0, 0);
    }else if(m_autoSelected == Paths.DALTSHOOTTWICE){
      autonomousBasePID.periodic(0, 146, 48, 0, 0, 0, 0);
    }

    //shoot subsystem's periodic is called in autonomous base
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //transitionSubsystem.init();
    //shootSubsystem.setState(ShootState.OFF);
    //transitionSubsystem.setState(TransitionState.OFF);
    //sensorSubsystem.setState(SensorStates.OFF);
    //intakeSubsystem.setState(IntakeStates.OFF);
    //sensorSubsystem.setup();
    //climberSubsystem.init();
    //shootSubsystem.init();
    driveSubsystem.setBrakeMode();
    turretSubsystem.init();
    limelightSubsystem.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){
    //driveSubsystem.drive();
    //shootSubsystem.periodic(false);
    //sensorSubsystem.periodic();
    //climberSubsystem.periodic(); //uncomment for real
    //intakeSubsystem.periodic();
    limelightSubsystem.periodic();
    turretSubsystem.periodic();

    //transition
    /*
    if(OI.transition.get()){
      transitionSubsystem.setState(TransitionState.ON);
      intakeSubsystem.setState(IntakeStates.INTAKING);
      if(TransitionSubsystem.state != TransitionState.ROTATE){ //this seems useless given we just turned it on? should we rethink sensor stuff -katherine
        sensorSubsystem.setState(SensorStates.ON);
      } 
    } else if(OI.transitionStop.get()){
      sensorSubsystem.setState(SensorStates.OFF);
      transitionSubsystem.setState(TransitionState.OFF);
      intakeSubsystem.setState(IntakeStates.OFF);
    } else if (OI.reverseIntakeTransition.get()){
      transitionSubsystem.setState(TransitionState.REVERSE);
      intakeSubsystem.setState(IntakeStates.OUTTAKING);
      sensorSubsystem.setState(SensorStates.OFF);
    } 
    */
    //shooter
    /* 
    if (OI.shooter.get()){
      ShootSubsystem.preShootSavedMillis = System.currentTimeMillis();
      transitionSubsystem.resetTareEncoder();
      shootSubsystem.setState(ShootState.SHOOT1);
    } else if(OI.shooterOff.get()){
      shootSubsystem.setState(ShootState.OFF);
    } else if(OI.warmUpShooter.get()){
      shootSubsystem.setState(ShootState.START);
    } else if (OI.reverseShoot.get()){
      shootSubsystem.setState(ShootState.REVERSE);
    }
    */
    //climber
    /*if(OI.climberActuate.get()){ 
      System.out.println("Actuating");
      climberSubsystem.setStateClimber(ClimberSubsystem.ClimberStates.ACTUATING);
    } else if(OI.climberRetract.get()){ 
      climberSubsystem.setStateClimber(ClimberSubsystem.ClimberStates.RETRACTING);
    }*/

    //turret
    /*
    if(OI.turretOn.get()){ 
      System.out.println("we took out this feature");
      //turretSubsystem.setTurretState(TurretSubsystem.TurretStates.TURNING);
    } else if(OI.turretOff.get()){ 
      turretSubsystem.setTurretState(TurretSubsystem.TurretStates.OFF);
    }
    */
    //all mechanism
    // if (OI.allMech.get()){
    //   shootSubsystem.setState(ShootState.START); 
    //   transitionSubsystem.setState(TransitionState.ON);
    //   intakeSubsystem.setState(IntakeStates.INTAKING);
    //   sensorSubsystem.setState(SensorStates.OFF);
    // } 
    /*
    if (OI.allMechOff.get()){
      shootSubsystem.setState(ShootState.OFF); 
      transitionSubsystem.setState(TransitionState.OFF);
      intakeSubsystem.setState(IntakeStates.OFF);
      sensorSubsystem.setState(SensorStates.OFF);
    }

    if(OI.shootOnce.get()){
      ShootSubsystem.isTwice = false;
    } else if(OI.shootTwice.get()){
      ShootSubsystem.isTwice = true;
    }
    */
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    //limelightSubsystem.init();
    //turretSubsystem.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //double selectedPipeline = pipelineChooser.getSelected();
    //limelightSubsystem.init(selectedPipeline);
   //driveSubsystem.driveTank(0.3, 0.3);
   //turretSubsystem.turnToAnglePD(150.0);
   //turretSubsystem.testTurretMotor();
   //driveSubsystem.driveTank(-0.1, -0.1);
   //turretSubsystem.periodic();
   //limelightSubsystem.periodic();
   //autonomousBasePID.turnToAnglePDVision(45);
   driveSubsystem.driveTank(0.3, 0.3);
  }
}
