package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;



public class newTurretSubsystem{
    public static final double TSPEED = 0.2; //change speed //changed on 6/22 to 0.10, originally 0.36
    private static TalonSRX turretMotor = new TalonSRX(RobotMap.TURRET);
    private double tareEncoder = 0.0;
    public static double KP_ANGLE = 0.00002;
    public static double KD_ANGLE = 0.00001;
    public static double MAXVOLT = 0.4;
    public static double MINVOLT = 0.2;
    private static final double MAXTICKS = 2048*0.26*0.1090278*180;
    public static double ticksWeWantToGo;
    
    public void init(){
        //turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Coast);
        //turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        resetEncoders();
    }
    
    public static enum TurretStates{
        CHECKTOTURN,
        TURNTOANGLE,
        NOTTURNING; 
    }

    public static TurretStates turretState = TurretStates.NOTTURNING;

    public void periodic(){
        if(turretState == TurretStates.CHECKTOTURN){
            System.out.println("checking angle");
            checkTicksForAngle(ticksWeWantToGo);
        } else if(turretState == TurretStates.TURNTOANGLE){
            System.out.println("turning to angle: " + ticksWeWantToGo);
            turnToAnglePD(ticksWeWantToGo);
        } else {
            motorsOff();
        }
    }

    public void setTurretState(TurretStates state){
        turretState = state;
    }

    public void motorsOff(){
        turretMotor.set(ControlMode.PercentOutput, 0.0);
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

    public void checkTicksForAngle(double ticksWeWantToGo){
        if(Math.abs(getTicks() + ticksWeWantToGo) < MAXTICKS){
            setTurretState(TurretStates.TURNTOANGLE);
        }else{
            ticksWeWantToGo = (MAXTICKS-getTicks())*Math.signum(ticksWeWantToGo);
            setTurretState(TurretStates.TURNTOANGLE);
            //turnToAnglePD((MAXTICKS-getTicks())*Math.signum(ticksWeWantToGo));
        }
    }

    public void turnToAnglePD(double ticks){
       // driveSubsystem.driveTank(0, 0);
        double velocity = getRate(); // get rate of change of gyro
        double error = ticks - getTicks(); // where we want to be - where we currently are (in degrees)
        System.out.println("error: " + error);
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
        if(Math.abs(error)<=580.0){//580.0){
            turretMotor.set(ControlMode.PercentOutput, 0.0);
            setTurretState(TurretStates.NOTTURNING);
        }else{
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