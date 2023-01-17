package us.ilite.robot.modules;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;


public class PracticeModule extends Module {

    private final NetworkTable mIntakeTable;
    private final NetworkTable mButtonTable;
    private TalonFX mMotor;




    public PracticeModule() {
        mMotor = new TalonFX(9);
        mIntakeTable = NetworkTableInstance.getDefault().getTable("intake");
        mButtonTable = NetworkTableInstance.getDefault().getTable("button being pressed");
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
        
        setIntakeTableValue("desiredpct", EIntakeData.DESIRED_ROLLER_pct);
        setIntakeTableValue("rollerpct", EIntakeData.ROLLER_PCT);

        setButtonTableValue("a_btn", ELogitech310.A_BTN);

    }

    private void setIntakeTableValue(String pEntry, EIntakeData pEnum)
    {
        mIntakeTable.getEntry(pEntry).setNumber(db.intake.get(pEnum));
    }

    private void setButtonTableValue(String pEntry, ELogitech310 pEnum)
    {
        mButtonTable.getEntry(pEntry).setNumber();
    }



}
