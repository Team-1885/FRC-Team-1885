package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EIntakeData.ROLLER_PCT;

public class PracticeSpinMotor extends Module{

    private final NetworkTable mTable;
    private TalonFX mMotor;

    public PracticeSpinMotor() {
        mMotor = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("practice");
    }

    @Override
    public void modeInit(EMatchMode pMode){
        //runs once when the robot is enabled
    }

    @Override
    public void readInputs() { // set current values that you get from your encoders
        db.intake.set(ROLLER_PCT, mMotor.getMotorOutputPercent());
    }

    public void setOutputs() { //set the hardware feature (in this case a motor) to your desiered value
        //percent output example:
        double percentSpeed = db.intake.get(EIntakeData.DESIRED_ROLLER_pct); // -1 -> 1
        mMotor.set(TalonFXControlMode.Velocity, percentSpeed);

        mTable.getEntry("desired percent output").setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mTable.getEntry("actual percent output").setNumber(db.intake.get(EIntakeData.ROLLER_PCT));


        //velocity example:
//        double velocity = db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s); //feet per second
//        mMotor.set(TalonFXControlMode.Velocity, velocity);

    }
}





