package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.Pigeon;

public class PigeonController {
    private Clock mClock;
    private Pigeon mPigeon;
    private Data db = Robot.DATA;

    public void initialize()
    {
        mClock = new Clock();
        mPigeon = new Pigeon(mClock, 30);

    }

    public void updateImpl()
    {
        mClock.update();

        //db.imu.get(EGyro.YAW_DEGREES)

    }
}
