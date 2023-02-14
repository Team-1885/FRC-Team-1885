package us.ilite.robot.modules;
import us.ilite.common.types.EMatchMode;

public class SamPositionControl extends Module{
    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public SamPositionControl(){

    }

    @Override
    public void modeInit(EMatchMode mode){

    }

    @Override
    public void readInputs(){

    }

    @Override
    public void setOutputs() {
        private double ticksToClimberDegrees(double pTicks) { return pTicks / 2048 * kClimberRatio * 360;
        }
        private double climberDegreesToTicks(double pDegrees) { return pDegrees * 2048 / kClimberRatio / 360;
        }
        }
    }

