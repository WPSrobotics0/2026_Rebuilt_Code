package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


//import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.SubsystemConstants;

public class FeederSubsystem  extends SubsystemBase {

    private final SparkMax m_FeederMotorLeft = new SparkMax(SubsystemConstants.kFeederLeftId, MotorType.kBrushless);
    private final SparkMax m_FeederMotorRight = new SparkMax(SubsystemConstants.kFeederRightId, MotorType.kBrushless);
    private final SparkMax m_FlipperMotor = new SparkMax(SubsystemConstants.kFeederFlipperId, MotorType.kBrushless);
    public FeederSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_FeederMotorLeft.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_FeederMotorRight.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_FlipperMotor.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {

    }
    public void setFeederSpeed(double speed) {
        m_FeederMotorLeft.set(speed);
        m_FeederMotorRight.set(speed);
    }
    public void setFlipperSpeed(double speed){
        //negative speed
        m_FlipperMotor.set(speed);
       
    }
    public void setFeederLeftSpeed(double speed) {
        m_FeederMotorLeft.set(-1*speed);
    }
    public void setFeederRightSpeed(double speed) {
        m_FeederMotorRight.set(-1*speed);
    }
    
}
