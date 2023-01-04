package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class TransitionSubsystem {
    public static TalonFX transitionMotor = new TalonFX(RobotMap.TRANSITION);
    public static double tareEncoder = transitionMotor.getSelectedSensorPosition();
    public static final double INSPEED = 0.2;
    public static final double OUTSPEED = 0.5;
    public static final double RSPEED = 0.5;
    public static final double ERORRTOLTOP = -620; //-640
    public static final double ERORRTOLBOTTOM = 0; //10
    public static final double TROTATION = 0.5;
    public static final double PTRANSITION = 0.00008; 
    public static final double DTRANSITION = 0.000001; 

    public TransitionSubsystem(){}

    public void resetTareEncoder(){
        tareEncoder = transitionMotor.getSelectedSensorPosition();
    }

    public static enum TransitionState{
        ON,
        ROTATE,
        OFF,
        STOP_PD,
        REVERSE;
    }
    
    public static TransitionState state = TransitionState.OFF;

    public void init(){
        transitionMotor.setNeutralMode(NeutralMode.Brake);
        System.out.println("switching to brake mode");
    }

    public void periodic(){
        if(state == TransitionState.ON){ 
            setMotor();
        } else if(state == TransitionState.ROTATE){
            rotateTransitionMotors(TROTATION);
        } else if(state == TransitionState.STOP_PD){
            stopPD(SensorSubsystem.setpoint);
        } else if(state == TransitionState.REVERSE){
            reverseTransition();
        }else {
            stopMotor();
        }
    }

    public void setState(TransitionState newState){
        state = newState;
    }
    
    public void setMotor(){
        transitionMotor.set(ControlMode.PercentOutput, INSPEED);   
    }

    public void setMotorOut(){
        transitionMotor.set(ControlMode.PercentOutput, RSPEED); 
    }

    public void stopPD(double setPoint)
    {   
        double velocity = getTransitionVelocity(); // time = per 100ms
        //System.out.println("Velocity: " + velocity);
        //System.out.println("setPoint transition: " + setPoint);
        double error = setPoint - getTicks();
        double voltage = (error*PTRANSITION) + (velocity*DTRANSITION);
        //System.out.println("voltage calculated: " + voltage);

        //max limiter
        if(Math.abs(voltage) > 0.5) {
            voltage = Math.signum(voltage)*0.5;
        }

        //minimum
        if(Math.abs(voltage) < 0.05) {
            voltage = 0;
        } else if (Math.abs(voltage) < 0.2) {
            voltage = 0.2 * Math.signum(voltage);
        }
    
        //error tolerance
        if (error < ERORRTOLTOP){ 
           transitionMotor.set(ControlMode.PercentOutput, Math.min(voltage, -voltage));
            //System.out.println("above setpoint" + voltage);
        }else if(error > ERORRTOLBOTTOM){ 
            transitionMotor.set(ControlMode.PercentOutput, voltage);
            //System.out.println("below setpoint" + voltage);
        }else{
            transitionMotor.set(ControlMode.PercentOutput, 0.0);
            ShootSubsystem.hasRotated = true;
        }

        //System.out.println("voltage final: " + voltage);
    }

    public double getTicks(){
        return transitionMotor.getSelectedSensorPosition() - tareEncoder; 
    }

    public double getTransitionVelocity(){
        return transitionMotor.getSelectedSensorVelocity(); //ticks per 100ms
    }
    
    public double getTransitionPosition(){
        return transitionMotor.getSelectedSensorPosition();
    }

    //call resetTareEncoder(); method before you call the rotateTransitionMotors method
    public void rotateTransitionMotors(double rotations){
        /* In the line below: 
        "rotations*2048" is the amount of rotations in ticks.
        "transitionMotor.getSelectedSensorPosition()" is the current position in ticks. We subtract tareEncoder
        to find t`he position in ticks relative to the starting position */
        if(rotations*2048 > transitionMotor.getSelectedSensorPosition() - tareEncoder){
            transitionMotor.set(ControlMode.PercentOutput, OUTSPEED);    
            System.out.println("the motor is on!");
        } else {
            transitionMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void reverseTransition(){
        transitionMotor.set(ControlMode.PercentOutput, -INSPEED);
    }

    private void stopMotor() {
        transitionMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
}
