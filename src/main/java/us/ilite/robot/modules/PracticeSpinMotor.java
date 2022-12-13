package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.SparkMaxFactory;

public class PracticeSpinMotor extends Module{
    private TalonFX mMotor;
    public PracticeSpinMotor() {
        mMotor = new TalonFX(0); //change id to actual id: maybe? Settings.HW.CAN.kINRoller
    }

    @Override
    public void modeInit(EMatchMode pMode){
        //runs once when the robot is enabled
    }

    @Override
    public void readInputs() { // set current values that you get from your encoders
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent()); //maybe: mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM) / kMaxFalconSpeed
    }

    public void setOutputs() { //set the hardware feature (in this case a motor) to your desiered value
        //percent output example:
        double percentSpeed = db.intake.get(EIntakeData.DESIRED_ROLLER_pct); // -1 -> 1
        mMotor.set(TalonFXControlMode.PercentOutput, percentSpeed);

        //velocity example:
        double velocity = db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s); //feet per second
        mMotor.set(TalonFXControlMode.Velocity, velocity);

    }
}





