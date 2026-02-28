// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

//import static edu.wpi.first.units.Units.Volt;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
public class ShootCommand extends Command {


  private ShooterSubsystem m_shooterSubsystem;
  private ElevatorSubsystem m_ElevatorSubsystem;
  private SpindexerSubsystem m_SpindexerSubsystem;
  private double m_targetZ; 
  private double m_tid;
  private double m_Forward;
  private double kSpeedKp = 0.125;
  private double kSpeedKi = 0;
  private double kSpeedKd = 0;
  private int ticks;
  //private double kRotationKd = 0;
  
  private Supplier<Double> m_shootTargetSpeed;

  private double m_driveForwardTarget;
  PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

  //private boolean m_tidFound = false;

  
  //private NetworkTable m_table;

  public ShootCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, SpindexerSubsystem spindexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_ElevatorSubsystem=elevatorSubsystem;
    m_SpindexerSubsystem=spindexerSubsystem;
    addRequirements(m_shooterSubsystem, m_ElevatorSubsystem, m_SpindexerSubsystem);

    //NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //m_table = inst.getTable("Shoot");
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_table=NetworkTableInstance.getDefault().getTable("limelight");
    double tid = LimelightHelpers.getFiducialID("limelight");
    m_tid=tid;
    ticks =0;
    //tweak on a per id basis, just a generic value for april tag
    m_targetZ =0;

    

    m_xSpeedController.setSetpoint(m_targetZ);

  }

  
  private double forward(){
    double kP=.1;
    m_Forward=LimelightHelpers.getTY("limelight") *kP;

    return m_xSpeedController.calculate(m_Forward)*(1);
  }

  private double calcDist(){
    double targetAngleOffset=LimelightHelpers.getTY("limelight") ;
    double limelightAngle=180-132;//132
    double limelightLensHeightInches = 6.0; 
    double goalHeightInches = 44.25;
    double angleToGoalDegrees = limelightAngle + targetAngleOffset;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
    return(goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  private double calcSpeed(double distance){
    //pythagorean (placeholder, I know not accurate at all) for speed
    double dist=calcDist();
    double distfromshooter=dist-42;
    double height= 44.25-6.0;
    double hypot=Math.sqrt((dist*dist)+(height*height));
    //seconds to reach target
    double seconds=5.0;
    double velocity = (2*distfromshooter)/3;
    //velocity= inches/seconds
    SmartDashboard.putNumber("dist", dist);
    SmartDashboard.putNumber("hypotinuse", hypot);
    SmartDashboard.putNumber("velocity", velocity);
    return velocity;
  }
  private double calcMotorVolts(double velocity){
    //wheels rpm
    double wheelDiameter=4.0;
    double wheelrpm=(velocity*60)/(Math.PI*wheelDiameter);
    
    double maxMotorrpm=5676.0; 

    maxMotorrpm=maxMotorrpm/16.0;

    //change to mapVolt change speed
    double MapVolt=3.0; 

    double Volts=(wheelrpm/maxMotorrpm)*MapVolt;
    
    SmartDashboard.putNumber("wheel rpm", wheelrpm);
    SmartDashboard.putNumber("motor volts", Volts);
   
    if(Volts<= 1.0){
      
         return Volts;
    } else {
      return 1.0;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_tid = LimelightHelpers.getFiducialID("limelight");
  
    SmartDashboard.putNumber(" command tid", m_tid);
   
    if (m_tid!=-1 && m_tid!=0) {
      //FORWARD SPEED
      m_driveForwardTarget=forward(); 

      SmartDashboard.putBoolean("isValidId", true);
      SmartDashboard.putNumber("drivespeed", m_driveForwardTarget);
      double ballVelocity = calcSpeed(calcDist());
      m_shootTargetSpeed =()->calcMotorVolts(ballVelocity);
      SmartDashboard.putNumber("target turret speed", m_shootTargetSpeed.get());
      m_shooterSubsystem.setIntakeSpeed(m_shootTargetSpeed.get());
      
      if(ticks>50){
        m_ElevatorSubsystem.setIntakeSpeed(0.5);
        if(ticks>55){
          m_SpindexerSubsystem.Spin(0.5);
        }
      }
      ticks++;
    }// else {
      //SmartDashboard.putBoolean("isValidId", false);
      //m_tidFound=false;
      //m_turretSubsystem.drive(0, 0, 0, true);
    //}
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.setIntakeSpeed(0.0);
    m_SpindexerSubsystem.Spin(0.0);
    m_shootTargetSpeed = ()->0.0;
    m_shooterSubsystem.setIntakeSpeed(m_shootTargetSpeed.get());
    //m_turretSubsystem.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
