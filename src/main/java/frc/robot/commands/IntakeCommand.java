package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeCommand extends Command {
   private IntakeSubsystem m_intake;
   private double m_speed=0.0;
   private double gearratio=12.0;
   public IntakeCommand(IntakeSubsystem intake, double speed){
    addRequirements(intake);
    m_intake = intake;
    m_speed=speed;
  
   }
    //only runs once
    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(()->m_speed);
    }
//everyt time it is ran
    @Override
  public void execute() {
    m_intake.setIntakeSpeed(()->m_speed);
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
