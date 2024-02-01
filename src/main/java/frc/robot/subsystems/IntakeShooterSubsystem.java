package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.TalonFx;
import frc.robot.motorcontrol.configurations.TalonFxConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeShooterSubsystem  extends SubsystemBase{

    //private final TalonFx m_intakeMotor = new TalonFx(Constants.Example.motorID, Constants.Example.motorRatio, TalonFx.makeDefaultConfig());
    private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(0);
    //private final TalonFx m_shooterMotor = new TalonFx(Constants.Example.motorID, Constants.Example.motorRatio, TalonFx.makeDefaultConfig());
    private final WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(1);
    public IntakeShooterSubsystem() {
      //Body
      //Show scheduler status in SmartDashboard.
      SmartDashboard.putData(this);

    }
    public void IntakeMotorStartSpin() {
      m_intakeMotor.set(1.0);
    }
  
    public void IntakeMotorReverseSpin() {
      m_intakeMotor.set(-1.0);
    }
  
    public void IntakeMotorStopSpin() {
      m_intakeMotor.set(0.0);
    }
    public void ShooterMotorStartSpin() {
      m_shooterMotor.set(1.0);
    }
  
    public void ShooterMotorReverseSpin() {
        m_shooterMotor.set(-1.0);
    }
  
    public void ShooterMotorStopSpin() {
      m_shooterMotor.set(0.0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    // --- BEGIN STUFF FOR SIMULATION ---
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    // --- END STUFF FOR SIMULATION ---
  }
    

