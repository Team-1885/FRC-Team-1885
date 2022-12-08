package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.SparkMaxFactory;

public class PracticeSpinMotor extends Module{
    //private CANSparkMax mMotor;
    private TalonFX mMotor;
    public PracticeSpinMotor() {
        mMotor = new TalonFX(0); //change id to actual id: maybe? Settings.HW.CAN.kINRoller
    }

    @Override
    public void modeInit(EMatchMode pMode){

    }

    @Override
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent()); //maybe: mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM) / kMaxFalconSpeed
    }

    public void setOutputs() {
        double speed = db.intake.get(EIntakeData.DESIRED_ROLLER_pct);
        mMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
}
