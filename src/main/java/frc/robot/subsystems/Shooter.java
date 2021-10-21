
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;



import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;



public class Shooter extends SubsystemBase{
    public static WPI_TalonSRX masterMotor;
    public static WPI_TalonSRX slaveMotor;
    public Encoder shooterEncoder;

    private double kP = 0.0000261; //proportional
    private double kI = 0; //integral
    private double kD = 0.0; //derivative
    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.49, 0.656, 0.00238);
    private PIDController pid = new PIDController(kP,kI,kD);
    
    private double ENCODER_EDGES_PER_REV = 4096 / 4.;
    private double encoderConstant = (1 / ENCODER_EDGES_PER_REV);
    
    

    double yaw;
    double pitch;
    double skew;
    Transform2d pose;
    Translation2d  translation;

    double range;

    double targetAngle;

    public Shooter(){
        masterMotor = new WPI_TalonSRX(4);
        slaveMotor = new WPI_TalonSRX(9);
        shooterEncoder = new Encoder(0, 1, false);
        shooterEncoder.setDistancePerPulse(encoderConstant);
        
        slaveMotor.setInverted(false);
        masterMotor.setInverted(false);
        //slaveMotor.follow(masterMotor);

        //shooterEncoder.setDistancePerPulse(encoderConstant);
        pid.setTolerance(100);
    }
    public void setShooter(double percentage) {
      //masterMotor.set(percentage);
      masterMotor.setVoltage(percentage*13);
      slaveMotor.setVoltage(percentage*13);
      //slaveMotor.set(percentage);
    }

    public double getRPM(){
        return shooterEncoder.getRate() * 60.;
    }

    public void stop(){
        setShooter(0);
    }
    public boolean isAtRPM(int RPM){
        if(Math.abs(getRPM() - RPM) <= 100){
            return true;
        }
        return false;
    }

    // ? Feedforward
    public void setRPM(int rpm){
        double ffOutput = feedforward.calculate(rpm);
        double pidOutput = pid.calculate(getRPM(), rpm);
        SmartDashboard.putNumber("Shooter FF  Output", ffOutput);
        SmartDashboard.putNumber("Shooter PID Output", pidOutput);
        setShooter(MathUtil.clamp(pidOutput, -1, 1));
    }

    public void runBackwards(){
        setShooter(-0.2);
    }
        
    
}
