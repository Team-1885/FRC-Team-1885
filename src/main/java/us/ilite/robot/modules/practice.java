package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.drive.EDriveData;
import static us.ilite.common.types.ELimelightData.LED_MODE;
import static us.ilite.common.types.ELimelightData.PIPELINE;

public class practice extends Module {

private TalonFX mMotor;
private double speed;

public practice(){
    mMotor = new TalonFX(9);
    mTable = NetworkTableInstance.getDefault().getTable("intake");
}

@Override
public void modeInit(EMatchMode pMode){

}
    private final NetworkTable mTable;

@Override
public void readInputs(){
db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
}

@Override
public void setOutputs(){
    speed = db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s);
    mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    setNetworkTableValue("intake", EIntakeData.DESIRED_ROLLER_pct);
}

    private void setNetworkTableValue(String pEntry, EIntakeData pEnum) {
        mTable.getEntry(pEntry).setNumber(db.intake.get(pEnum));
    }

}