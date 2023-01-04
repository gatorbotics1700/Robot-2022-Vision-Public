package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimelightSubsystem.LimelightStates;



public class TurretSubsystem{
    public static final double TSPEED = 0.15; //change speed //changed on 6/22 to 0.10, originally 0.36
    private static TalonSRX turretMotor = new TalonSRX(RobotMap.TURRET);
    private double tareEncoder = 0.0;
    public static double KP_ANGLE = 0.00003; //0.0000003;//0.000000003;
    public static double KD_ANGLE = 0.005;
    public static double MAXVOLT = 0.4;
    public static double MINVOLT = 0.2;
    private static final double MAXTICKS = 2048*10*0.00277778*180;
    private static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private static double turretTx_0 = 0.0;
    private static double turretTx_1 = 0.0;
    public void init(){
        //turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Coast);
        //turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        resetEncoders();
        setTurretState(TurretStates.OFF);
    }
    
    public static enum TurretStates{
        TURNINGZERO,
        TURNINGONE,
        TURNINGBACK,
        OFF; 
    }

    public static TurretStates turretState = TurretStates.OFF;

    public void periodic(){
        if (turretState == TurretStates.TURNINGZERO) {
            System.out.println("turningzero");
            System.out.println("tx_0 in ticks: " + turretTx_0*2048.0*10.0*0.00277778);
            System.out.println("tx_0 is: " + turretTx_0);
            turnToAnglePD(turretTx_0*2048.0*10.0*0.00277778);
            //checkTicksForAngle(turretTx_0*2048.0*10.0*0.00277778);
        }else if (turretState == TurretStates.TURNINGONE) {
            System.out.println("getTicks(): " + getTicks());
            System.out.println("turningone");
            System.out.println("tx_1 in ticks: " + turretTx_1*2048.0*10.0*0.00277778);
            turnToAnglePD(turretTx_0*2048.0*10.0*0.00277778);
        } else if (turretState == TurretStates.OFF){
            motorsOff();
        } else if(turretState == TurretStates.TURNINGBACK){
            System.out.println("turningback");
            turnToAnglePD(turretTx_0*2048.0*10.0*0.00277778);
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

    public void setTurretTx_0(double value){
        turretTx_0 = value;
    }

    public void setTurretTx_1(double value){
        turretTx_1 = value;
    }

    public void checkTicksForAngle(double ticksWeWantToGo){
        //System.out.println(ticksWeWantToGo);
        if(Math.abs(getTicks() + ticksWeWantToGo) < MAXTICKS){ //deal with tare
            //resetEncoders();
            turnToAnglePD(ticksWeWantToGo);
            //System.out.println("yas");
        }else{
            //resetEncoders();
            turnToAnglePD((2*MAXTICKS-Math.abs(ticksWeWantToGo))*Math.signum(-ticksWeWantToGo));
        }
    }

    public void turnToAnglePD(double ticks){
       // driveSubsystem.driveTank(0, 0);
       System.out.println("ticks at start of turnToAnglePD: " + ticks);
        double velocity = getRate(); // get rate of change of gyro
        double error = ticks - getTicks(); // where we want to be - where we currently are (in ticks)
        System.out.println("Our ticks: " + getTicks()); 
        double voltage = (error*KP_ANGLE) + (velocity*KD_ANGLE);
        System.out.println("error: "+ error);
        //System.out.println("Original voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        System.out.println("Our Voltage: " + voltage);
        
        if(Math.abs(error) <= 350.0){//~2.988 degrees = 170 ticks
            System.out.println("inside deadband!!!!!!!");
            if(turretState == TurretStates.TURNINGZERO){
                limelightSubsystem.setState(LimelightStates.OFF);//used to be scanone
                resetEncoders();
                setTurretState(TurretStates.OFF);
            }else if (turretState == TurretStates.TURNINGONE){
                if(limelightSubsystem.yDistanceFromIdeal()<6.0 && limelightSubsystem.yDistanceFromIdeal()>-6.0){
                    System.out.println("going onto SHOOT");
                    limelightSubsystem.setState(LimelightStates.OFF);
                    resetEncoders();
                    setTurretState(TurretStates.OFF);
                 }else{
                     System.out.println("going onto CENTER");
                     resetEncoders();
                     //setTurretState(TurretStates.TURNINGBACK); 
                     limelightSubsystem.setState(LimelightStates.OFF);
                     setTurretState(TurretStates.OFF);
                 }
            }else if (turretState == TurretStates.TURNINGBACK){
                limelightSubsystem.setState(LimelightStates.MOVE);
                setTurretState(TurretStates.OFF);
            }
        } else {
            turretMotor.set(ControlMode.PercentOutput, voltage); //left, right
        }
    

    }
    public double getTareEncoder(){
        return tareEncoder;
    }
    public void testTurretMotor(){
        turretMotor.set(ControlMode.PercentOutput, 0.2);





















    }
}