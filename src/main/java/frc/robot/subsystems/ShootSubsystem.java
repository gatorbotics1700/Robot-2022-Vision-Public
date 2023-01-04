package frc.robot.subsystems;

import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.SensorSubsystem.SensorStates;
import frc.robot.subsystems.TransitionSubsystem.TransitionState;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;
import frc.robot.Autonomous.AutonomousBase;
import frc.robot.Autonomous.AutonomousBase.Paths;

public class ShootSubsystem{
    public static TalonFX shooter_1 = new TalonFX(RobotMap.SHOOTER_1);
    public static TalonFX shooter_2 = new TalonFX(RobotMap.SHOOTER_2);

    public static final double SHOOTRPM = 1940; //1820.0;  //1860; //2233.0;
    public static final double SHOOTRPMAUTO = 1850;
    public static final double SHOOTSPEED_AUTO = 0.38; 
    public static final double SHOOTSPEEDFOR2 = 0.36;
    public static final double SHOOTSPEED_TELE = 0.36; 
    public static final double SHOOTSPEED_AUTO_ALT = 0.37; // finetuned ish on 4/8 fri
    public static final double TROTATION = 1.5;
    public static final double MSSHOOT = 900;
    
    public double tareEncoder;
    public static double preShootSavedMillis;
    public static Boolean isTwice = false; //for teleop
    public static Boolean hasShot; //for auto
    public static Boolean hasRotated; //for teleop shooting two balls

    public static TransitionSubsystem transitionSubsystem = new TransitionSubsystem();
    public static SensorSubsystem sensorSubsystem = new SensorSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    
    public static enum ShootState{
        START,
        SHOOT1,
        SHOOT2,
        OFF,
        REVERSE;
    }

    public void init(){
        isTwice = false;
        hasShot = false;
        hasRotated = false;
        tareEncoder = 0.0;
        preShootSavedMillis = System.currentTimeMillis();
        shooter_1.setNeutralMode(NeutralMode.Coast);
        shooter_2.setNeutralMode(NeutralMode.Coast);
    }

    public static ShootState state = ShootState.OFF;
    
    public void periodic(boolean isAuto){
        if (state == ShootState.START){
            runProperShot(isAuto);
        } else if (state == ShootState.SHOOT1){
            runProperShot(isAuto);
            System.out.println("istwice" + isTwice);
            if(isTwice){
                transitionSubsystem.setState(TransitionState.ROTATE); //set state to rotate
                System.out.println("has rotated" + hasRotated);
                System.out.println("wait:" + waitForIdealMotorSpeed(isAuto));
                if (hasRotated && waitForIdealMotorSpeed(isAuto) == true){ //replaced time with hasRotated
                    preShootSavedMillis = System.currentTimeMillis();
                    transitionSubsystem.resetTareEncoder();
                    setState(ShootState.SHOOT2);
                }
            }else{ //if we are shooting 1 ball at a time
                transitionSubsystem.setState(TransitionState.ON);
                sensorSubsystem.setState(SensorStates.OFF);
                if (isAuto){
                    System.out.println("current time: " + System.currentTimeMillis() + "pre shoot saved millis: " + preShootSavedMillis);
                    if (System.currentTimeMillis() >= preShootSavedMillis + MSSHOOT){
                        System.out.println("done w/ shooting timer -- moving onto start state");
                        transitionSubsystem.resetTareEncoder();
                        hasShot = true;
                        setState(ShootState.START);
                        hasRotated = false;
                        intakeSubsystem.setState(IntakeStates.INTAKING);
                        transitionSubsystem.setState(TransitionState.ON);
                        sensorSubsystem.setState(SensorStates.ON);  
                    }
                } else { //teleop
                    if (System.currentTimeMillis() >= preShootSavedMillis + MSSHOOT){
                        System.out.println("done w/ shooting timer -- moving onto start state");
                        transitionSubsystem.resetTareEncoder();
                        setState(ShootState.START);
                        hasRotated = false;
                        intakeSubsystem.setState(IntakeStates.INTAKING);
                        transitionSubsystem.setState(TransitionState.ON);
                        sensorSubsystem.setState(SensorStates.ON);  
                    }
                }

            }   
        } else if (state == ShootState.SHOOT2){
            runProperShot(isAuto);
            transitionSubsystem.setState(TransitionState.ON);
            if(System.currentTimeMillis() >= preShootSavedMillis + MSSHOOT){
                setState(ShootState.START);
                intakeSubsystem.setState(IntakeStates.INTAKING);
                transitionSubsystem.setState(TransitionState.ON);
                sensorSubsystem.setState(SensorStates.ON);
            }
        } else if(state == ShootState.REVERSE){
            reverseShooter();
        } else {
            shooterOff();
        }
    }   

    public void setState(ShootState newState){
        state = newState;
    }

    public void runProperShot(boolean isAuto){
        if (isAuto){
            if (AutonomousBase.isSwap) {
                runShooterMotor(SHOOTSPEED_AUTO_ALT);
            } else {
                runShooterMotor(SHOOTSPEED_AUTO);
            }
        } else if (isTwice){
            runShooterMotor(SHOOTSPEEDFOR2);
        } else {
            runShooterMotor(SHOOTSPEED_TELE); 
        }
    }

    public boolean waitForIdealMotorSpeed(boolean isAuto){
        if (isAuto){
            if(Math.abs(getRPM()) >= SHOOTRPMAUTO){
                return true;
             }
        } else {
            if(Math.abs(getRPM()) >= SHOOTRPM){
             return true;
            }
        }
       return false;
       // shooter1 motor rpm needs to be a conversion from ticks->rpm
    }

    public void runShooterMotor(double speed){
        shooter_1.set(ControlMode.PercentOutput, -speed);
        shooter_2.set(ControlMode.PercentOutput, speed);
    }

    public void reverseShooter(){
        shooter_1.set(ControlMode.PercentOutput, SHOOTSPEED_TELE);
        shooter_2.set(ControlMode.PercentOutput, -SHOOTSPEED_TELE);
    }

    public void shooterOff(){
        shooter_1.set(ControlMode.PercentOutput, 0.0);
        shooter_2.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetEncoders(){
        tareEncoder = shooter_1.getSelectedSensorPosition(); 
        // taring from 1 encoder's reading because all encoders increasing by same amount
    }

    public double getTicks(){
        return shooter_1.getSelectedSensorPosition() - tareEncoder; // same encoder as in "resetEncoders"
    }

    public double getRPM(){
        double shooterMotorRPM = ((shooter_1.getSelectedSensorVelocity()/2048/100)*1000*60);
        return shooterMotorRPM;
    }

}