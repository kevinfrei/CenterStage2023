package org.firstinspires.ftc.teamcode.driver.vl53l4cx;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

public class VL53L4CXDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    protected VL53L4CXDevice(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }
}
