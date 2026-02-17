package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAnglerSubsystem;
public class AngleShooterCommand extends Command {
   private ShooterAnglerSubsystem m_Angler;
   private Supplier<Double> m_speed = () -> 0.0;
   public AngleShooterCommand(ShooterAnglerSubsystem Angler){
    addRequirements(Angler);
    m_Angler = Angler;
    
    
   }
    //only runs once
    @Override
    public void initialize() {
        //m_Angler.Rotateshooter(()->1.0);
    }
//everyt time it is ran
    @Override
  public void execute() {
    //m_Angler.Rotateshooter(()->1.0);
    m_speed=()->m_Angler.m_AnglerEncoder.getPosition()/16.0;
    m_speed= ()->Math.abs(1-m_speed.get())/2.0;
    if (m_speed.get()>1.0){
      m_speed=()->1.0;
    }
    SmartDashboard.putNumber("angle rotate speed", m_speed.get());
    if (1-.05>m_Angler.m_AnglerEncoder.getPosition()/16.0) {
        
         m_Angler.setSpeed(m_speed);

        }
        else if (1+0.05<m_Angler.m_AnglerEncoder.getPosition()/16.0) {
          m_speed=()-> -1*m_speed.get();
            m_Angler.setSpeed(m_speed);
        }
        else{
            m_Angler.setSpeed(()->0.0);
        } 
  }
   @Override
  public void end(boolean interrupted) {
     m_Angler.setSpeed(()->0.0);
}
   @Override
  public boolean isFinished() {
    return false;
  }
}
