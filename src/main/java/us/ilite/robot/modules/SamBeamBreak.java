package us.ilite.robot.modules;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.DigitalBeamSensor;

import static us.ilite.common.types.EClimberData.SINGLE_BEAM_BROKEN;

public class SamBeamBreak extends Module{
    private DigitalBeamSensor mSingleBreak;
    boolean isBeamBroken = false;


    public SamBeamBreak(){
    // mSingleBreak = new DigitalBeamSensor(3); commented out temporarily
}

    @Override
    public void modeInit(EMatchMode mode){
        db.climber.set(SINGLE_BEAM_BROKEN, !mSingleBreak.isBroken());
}

    @Override
    public void readInputs(){
     
}

    @Override
    public void setOutputs(){

    }
}