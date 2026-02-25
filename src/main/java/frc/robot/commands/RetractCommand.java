package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
public class RetractCommand extends Command {
   private IntakeSubsystem m_intake;
   private double m_speed=1.0;
   private double m_Target=0.0;
   private double gearratio=16.0;
   public RetractCommand(IntakeSubsystem intake){
    addRequirements(intake);
    m_intake = intake;
  
   }
    //only runs once
    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(()->m_speed);
    }
//everyt time it is ran
    @Override
  public void execute() {
    m_intake.setIntakeSpeed(()->0.0);
  double speed = Math.abs(m_Target-m_intake.m_IntakeMotorRightEncoder.getPosition()/gearratio);
  if (speed>1.0){
    speed=1.0;
  }
  speed*=-1.0;
  m_intake.setRotate(speed);
  }
   @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(()->0.0);
}


   @Override
  public boolean isFinished() {
    return false;
  }
}
