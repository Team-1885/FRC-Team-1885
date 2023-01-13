package us.ilite.robot.modules;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.TalonSRXFactory;
import us.ilite.robot.modules.FalconDriveModule;

public class PracticeModule extends Module {

    private final NetworkTable mTable;
    private TalonFX mMotor;
    private double speed;
    private String pEtntry;


    public PracticeModule() {
        TalonFX mMotor = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("intake");
    }

    @Override
    public void modeInit(EMatchMode pMode){

    }



    @Override
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());

    }

    @Override
    public void setOutputs() {
        mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        
        setNetworkTableValue("desiredpct", EIntakeData.DESIRED_ROLLER_pct);
        setNetworkTableValue("rollerpct", EIntakeData.ROLLER_PCT);

    }

    private void setNetworkTableValue(String pEntry, EIntakeData pEnum)
    {
        mTable.getEntry(pEtntry).setNumber(db.intake.get(pEnum));
    }

}
