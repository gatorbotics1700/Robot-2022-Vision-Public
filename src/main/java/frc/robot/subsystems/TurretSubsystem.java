package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;



public class TurretSubsystem{
    public static final double TSPEED = 0.36; //change speed
    private static TalonSRX turretMotor = new TalonSRX(RobotMap.TURRET);
    private double tareEncoder = 0.0;
    public static double KP_ANGLE = 0.0;
    public static double KD_ANGLE = 0.0;
    public static double MAXVOLT = 0.4;
    public static double MINVOLT = 0.1;

    public void init(){
        //turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Coast);
        //turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        resetEncoders();
    }
    
    public static enum TurretStates{
        TURNING, 
        NOTTURNING; 
    }

    public static TurretStates turretState = TurretStates.NOTTURNING;

    public void periodic(){
        if (turretState == TurretStates.TURNING) {
            motorsOn();
        } else {
            motorsOff();
        }
    }
    public void motorsOn(){
        turretMotor.set(ControlMode.PercentOutput, TSPEED);
    }
    public void motorsOff(){
        turretMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public void setTurretState(TurretStates state){
        turretState = state;
    }
    public double getTicks(){
        return turretMotor.getSelectedSensorPosition() - tareEncoder; 
    }

    public void resetEncoders(){
        tareEncoder = turretMotor.getSelectedSensorPosition(); 
        // taring from 1 encoder's reading because all encoders increasing by same amount
    }

    public double getRate(){
        return turretMotor.getSelectedSensorVelocity();
    }

    public void turnToAnglePD(double ticks){
        double velocity = getRate(); // get rate of change of gyro
        double error = ticks - getTicks(); // where we want to be - where we currently are (in degrees)
        //System.out.println("Our Degrees: " + driveSubsystem.getDegrees()); 
        double voltage = (error*KP_ANGLE) + (velocity*KD_ANGLE);
        //System.out.println("Original voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        //System.out.println("Our Voltage: " + voltage);
        
        turretMotor.set(ControlMode.PercentOutput, voltage); //left, right
    
    }
    public double getTareEncoder(){
        return tareEncoder;
    }
}