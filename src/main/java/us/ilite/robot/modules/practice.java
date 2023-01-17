package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.EDriverInputMode;
import us.ilite.common.types.input.ELogitech310;

import static us.ilite.common.types.ELimelightData.LED_MODE;
import static us.ilite.common.types.ELimelightData.PIPELINE;

public class practice extends Module {

private TalonFX mMotor;
private double speed;

public practice(){
    mMotor = new TalonFX(9);
    mIntakeTable = NetworkTableInstance.getDefault().getTable("intake");
    mButtonTable = NetworkTableInstance.getDefault().getTable("button is pressed");
}

@Override
public void modeInit(EMatchMode pMode){

}
    private final NetworkTable mIntakeTable;

    private final NetworkTable mButtonTable;

@Override
public void readInputs(){
db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
}

@Override
public void setOutputs(){
    speed = db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s);
    mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    setIntakeTableValue("intake", EIntakeData.DESIRED_ROLLER_pct);
    setButtonTableValue("button is pressed", ELogitech310.A_BTN);
}

    private void setIntakeTableValue(String pEntry, EIntakeData pEnum) {
        mIntakeTable.getEntry(pEntry).setNumber(db.intake.get(pEnum));
    }

    private void setButtonTableValue(String pEntry, ELogitech310 pEnum) {
        mButtonTable.getEntry(pEntry).setNumber(db.driverinput.get(pEnum));
    }

}