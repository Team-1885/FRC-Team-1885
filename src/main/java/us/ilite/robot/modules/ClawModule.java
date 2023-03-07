package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.sensor.EClawData;

public class ClawModule extends Module {

    // ============================
    // CREATES TWO MOTORS FOR CLAW
    // ============================

    TalonFX mLeftMotor;
    TalonFX mRightMotor;

    NetworkTable mTable;

    public ClawModule() {
        // ========================================================================================
        // CREATES ALL THE MOTORS AND SETS ONE OF THE MOTORS TO BE INVERTED (SPIN THE OPPOSITE WAY
        // ========================================================================================\

        mLeftMotor = new TalonFX(19);
        mRightMotor = new TalonFX(20);
        mRightMotor.setInverted(true);

        // ====================================
        // CREATES A GLASS ENTRY FOR DEBUGGING
        // ====================================

        mTable = NetworkTableInstance.getDefault().getTable("claw");
    }
    @Override
    public void modeInit(EMatchMode mode) {

    }
    @Override
    public void readInputs() {
        //==========================================
        // SETS THE ACTUAL VELOCITY USING THE SENSOR
        //==========================================
        db.claw.set(EClawData.ACTUAL_VEL_ft_s,  mLeftMotor.getSelectedSensorVelocity() * IntakeModule.kFeetSpeedConversion);
        // db.claw.set(EClawData.ACTUAL_POS_DEG, ticksToClimberDegrees( mLeftMotor.getSelectedSensorPosition()));


    }
    @Override
    public void setOutputs() {
        // =============================================
        // SETS THE MOTORS VELOCITY TO DESIRED VELOCITY
        // =============================================

        mLeftMotor.set(TalonFXControlMode.Velocity, db.claw.get(EClawData.DESIRED_VEL_ft_s));
        mRightMotor.set(TalonFXControlMode.Velocity, db.claw.get(EClawData.DESIRED_VEL_ft_s));


        // ==================================================================================================
        // CREATES ENTRIES FOR GLASS TABLE, USED FOR DEBUGGING AND SEEING VALUES INCLUDED WITHIN THE ENTRIES
        // ==================================================================================================


        mTable.getEntry("DESIRED_VEL_ft_s").setNumber(db.claw.get(EClawData.DESIRED_VEL_ft_s));
        // mTable.getEntry("ACTUAL_POS_DEG").setNumber(db.claw.get(EClawData.ACTUAL_POS_DEG));
        mTable.getEntry("ACTUAL_VEL_ft_s").setNumber(db.claw.get(EClawData.ACTUAL_VEL_ft_s));
    }

    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * ReferenceModule.kClimberRatio * 360;
    }
}
