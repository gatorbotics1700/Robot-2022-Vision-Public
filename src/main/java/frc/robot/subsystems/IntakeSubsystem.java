package frc.robot.subsystems;
    
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;

public class IntakeSubsystem {
    public static final double FSPEED = 0.36; //siona changed from 0.33
    public static final double RSPEED = -0.7;
    private static TalonFX intakeMotor = new TalonFX(RobotMap.INTAKE);
    
    public void init(){
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    public static enum IntakeStates{
        INTAKING, 
        OFF,
        OUTTAKING; 
    }

    public static IntakeStates intakeState = IntakeStates.OFF;

    public void periodic(){
        if (intakeState == IntakeStates.INTAKING) {
            motorsOn();
        } else if (intakeState == IntakeStates.OUTTAKING) {
            motorsReverse();
        } else {
            motorsOff();
        }
    }

    public void motorsOn(){
        intakeMotor.set(ControlMode.PercentOutput, FSPEED);
    }

    public void motorsReverse(){
        intakeMotor.set(ControlMode.PercentOutput, RSPEED);
    }

    public void motorsOff(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void setState(IntakeStates state){
        intakeState = state;
    }
}

