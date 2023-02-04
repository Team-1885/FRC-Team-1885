package us.ilite.robot.commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import us.ilite.robot.modules.NeoDriveModule;

public class PigeonTest {
    // only controller need an initialize with no params
    private NeoDriveModule mDrive;
    private WPI_PigeonIMU gyro;

    public PigeonTest() {
        gyro = new WPI_PigeonIMU(0); // Pigeon is on CAN Bus with device ID 0
        mDrive = NeoDriveModule.getInstance();
    }


}
