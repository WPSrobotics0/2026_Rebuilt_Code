package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeLiftCommand extends Command {

  private IntakeSubsystem m_intake;
  private Supplier<Double> m_leftY;
  public IntakeLiftCommand(IntakeSubsystem Intake, Supplier<Double> leftY){
    addRequirements(Intake);
    m_intake = Intake;
    m_leftY=leftY;
  }
    
  @Override
  public void initialize() {
    m_intake.setRotate(m_leftY);
  }

  @Override
  public void execute() {
    double lowerBound=-3.0;
    double upperBound=7.0;
    if((m_intake.m_IntakeLiftMotorEncoder.getPosition() > lowerBound) && (m_intake.m_IntakeLiftMotorEncoder.getPosition() < upperBound)){
      m_intake.setRotate(m_leftY);
    } 
    else if(m_intake.m_IntakeLiftMotorEncoder.getPosition() < lowerBound){
      if(m_leftY.get() < -0.05){
        m_intake.setRotate(()->0.0);
      } 
      else{
        m_intake.setRotate(m_leftY);
      }
    } 
    else if(m_intake.m_IntakeLiftMotorEncoder.getPosition() > upperBound){
      if(m_leftY.get() > 0.05){
        m_intake.setRotate(()->0.0);
      } 
      else{
        m_intake.setRotate(m_leftY);
      }
    } 
    else{
      m_intake.setRotate(()->0.0);
    }
    
    SmartDashboard.putNumber("lift motor position", m_intake.m_IntakeLiftMotorEncoder.getPosition());
    SmartDashboard.putNumber("subDrive LeftY", m_leftY.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setRotate(()->0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}