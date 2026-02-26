package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.SubsystemConstants;

public class IntakeSubsystem  extends SubsystemBase {

    private final SparkMax m_IntakeMotor = new SparkMax(SubsystemConstants.kIntakeId, MotorType.kBrushless);
    private final SparkMax m_IntakeLiftMotor = new SparkMax(SubsystemConstants.kIntakeLiftId, MotorType.kBrushless);
    public SparkRelativeEncoder m_IntakeLiftMotorEncoder = (SparkRelativeEncoder) m_IntakeLiftMotor.getEncoder();
    public IntakeSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_IntakeMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_IntakeLiftMotor.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {

    }
    public void setIntakeSpeed(Supplier<Double> speed) {
        m_IntakeMotor.set(speed.get());
        //m_IntakeMotorRight.set(-1*speed.get());
        SmartDashboard.putNumber("Intake Speed", speed.get());
    }
    public void setRotate(double speed){
        m_IntakeLiftMotor.set(speed);
        SmartDashboard.putNumber("Intake Rotate Speed", speed);    }

}
