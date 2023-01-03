package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.drive.EDriveData;

public class practice extends Module {

private TalonFX mMotor;
private double speed;

public practice(){
    mMotor = new TalonFX(9);
}

@Override
public void modeInit(EMatchMode pMode){

}
@Override
public void readInputs(){
db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
}

@Override
public void setOutputs(){
    speed = db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s);
    mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
}
}