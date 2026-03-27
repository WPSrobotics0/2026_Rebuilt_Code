package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import java.util.function.Supplier;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShooterSubsystem  extends SubsystemBase {
    private final SparkMax m_moter = new SparkMax(SubsystemConstants.kShooterId, MotorType.kBrushless);
    private SparkClosedLoopController moterController=m_moter.getClosedLoopController();
    private Supplier<Double> m_speed;

    public ShooterSubsystem() {
        SparkMaxConfig moterConfig =new SparkMaxConfig();
        //tune later
        moterConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).inverted(false).closedLoop
        .p(0.0003).i(0).d(0).maxMotion.cruiseVelocity(5000).maxAcceleration(17500)
        .allowedProfileError(1);
        m_moter.configure(moterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        

        m_speed = () -> 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret Speed", m_speed.get());
        
    }

    public void setIntakeSpeed(double speed) {
        //m_moter.set(speed);
        moterController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        //SmartDashboard.putNumber("speed Incrementer for shooter", speed);
    }    
}