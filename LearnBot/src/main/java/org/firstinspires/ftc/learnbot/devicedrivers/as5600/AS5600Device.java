package org.firstinspires.ftc.learnbot.devicedrivers.as5600;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

// Just following along with the guide here:
// https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver
// Product brief: https://ams.com/as5600
// Datasheet accessible from that page
@I2cDeviceType
@DeviceProperties(name = "AS5600 Rotational Sensor", xmlTag = "AS5600")
public class AS5600Device extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x36);

    static class Constants {

        /* From the header file:
            //  setDirection: KBF - can't do this from the REV I2C bus alone
            const byte Register.CLOCK_WISE         = 0;  //  LOW
            const byte Register.COUNTERCLOCK_WISE  = 1;  //  HIGH
        */
        //  0.087890625;
        public static final float RAW_TO_DEGREES = 360.0f / 4096;
        //  0.00153398078788564122971808758949;
        public static final float RAW_TO_RADIANS = (float) ((Math.PI * 2.0) / 4096f);

        //  STATUS BITS
        public static final short MAGNET_HIGH = 0x08;
        public static final short MAGNET_LOW = 0x10;
        public static final short MAGNET_DETECT = 0x20;
        public static final byte MODE_DEGREES = 0;
        public static final byte MODE_RADIANS = 1;
        /*
            //  getAngularSpeed

            //  CONFIGURE CONSTANTS
            //  check datasheet for details

            //  setOutputMode
            const byte Register.OUTMODE_ANALOG_100 = 0;
            const byte Register.OUTMODE_ANALOG_90  = 1;
            const byte Register.OUTMODE_PWM        = 2;

            //  setPowerMode
            const byte Register.POWERMODE_NOMINAL  = 0;
            const byte Register.POWERMODE_LOW1     = 1;
            const byte Register.POWERMODE_LOW2     = 2;
            const byte Register.POWERMODE_LOW3     = 3;

            //  setPWMFrequency
            const byte Register.PWM_115            = 0;
            const byte Register.PWM_230            = 1;
            const byte Register.PWM_460            = 2;
            const byte Register.PWM_920            = 3;

            //  setHysteresis
            const byte Register.HYST_OFF           = 0;
            const byte Register.HYST_LSB1          = 1;
            const byte Register.HYST_LSB2          = 2;
            const byte Register.HYST_LSB3          = 3;

            //  setSlowFilter
            const byte Register.SLOW_FILT_16X      = 0;
            const byte Register.SLOW_FILT_8X       = 1;
            const byte Register.SLOW_FILT_4X       = 2;
            const byte Register.SLOW_FILT_2X       = 3;

            //  setFastFilter
            const byte Register.FAST_FILT_NONE     = 0;
            const byte Register.FAST_FILT_LSB6     = 1;
            const byte Register.FAST_FILT_LSB7     = 2;
            const byte Register.FAST_FILT_LSB9     = 3;
            const byte Register.FAST_FILT_LSB18    = 4;
            const byte Register.FAST_FILT_LSB21    = 5;
            const byte Register.FAST_FILT_LSB24    = 6;
            const byte Register.FAST_FILT_LSB10    = 7;

            //  setWatchDog
            const byte Register.WATCHDOG_OFF       = 0;
            const byte Register.WATCHDOG_ON        = 1;
        */

    }

    public enum Register {
        FIRST(0),

        //  CONFIGURATION REGISTERS
        ZMCO(0x0),
        ZPOS(0x1),
        MPOS(0x3),
        MANG(0x5),
        CONF(0x7),
        CONF_1(0x8),

        //  CONFIGURATION BIT MASKS - byte level
        CONF_POWER_MODE(0x03),
        CONF_HYSTERESIS(0x0C),
        CONF_OUTPUT_MODE(0x30),
        CONF_PWM_FREQUENCY(0xC0),
        CONF_SLOW_FILTER(0x03),
        CONF_FAST_FILTER(0x1C),
        CONF_WATCH_DOG(0x20),

        //  OUTPUT REGISTERS
        RAW_ANGLE(0x0C),
        ANGLE(0x0E),

        //  STATUS REGISTERS
        STATUS(0x0B),
        AGC(0x1A),
        MAGNITUDE(0x1B),
        BURN(0xFF),

        // Last register number:
        LAST(0x20);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
            Register.FIRST.bVal,
            Register.LAST.bVal - Register.FIRST.bVal + 1,
            I2cDeviceSynch.ReadMode.REPEAT
        );
        this.deviceClient.setReadWindow(readWindow);
    }

    private byte error;
    //  for getAngularSpeed()
    private ElapsedTime lastMeasurement;
    short lastAngle;
    //  for readAngle() and rawAngle()
    short offset;

    private byte[] buffer1;
    private byte[] buffer2;

    public AS5600Device(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.error = 0;
        this.lastMeasurement = new ElapsedTime();
        this.lastAngle = 0;
        this.offset = 0;
        this.buffer1 = new byte[1];
        this.buffer2 = new byte[2];
        // Enable reading all the registers in a batch to speed things up
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        // This apparently deals with USB cables getting unplugged
        super.registerArmingStateCallback(false);
        // Sensor starts off disengaged so we can change things like I2C address
        // Need to engage to start the device...
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "AS5600";
    }

    // Helpers named to enable easy translation from the original C++ code:
    protected byte readReg(final Register r) {
        return deviceClient.read8(r.bVal);
    }

    protected byte readReg(final Register r, int mask) {
        return (byte) (deviceClient.read8(r.bVal) & mask);
    }

    protected void writeReg(final Register r, byte b) {
        this.buffer1[0] = b;
        deviceClient.write(r.bVal, this.buffer1);
    }

    protected short readReg2(final Register r) {
        // TODO: sure would be nice if I could eliminate the allocation here
        return TypeConversion.byteArrayToShort(deviceClient.read(r.bVal, 2));
    }

    protected short readReg2(final Register r, int mask) {
        // TODO: sure would be nice if I could eliminate the allocation here
        return (short) (mask & TypeConversion.byteArrayToShort(deviceClient.read(r.bVal, 2)));
    }

    protected void writeReg2(final Register r, short b) {
        // Apparently I2C is big-endian
        this.buffer2[0] = (byte) ((b >> 8) & 0xFF);
        this.buffer2[1] = (byte) (b & 0xFF);
        deviceClient.write(r.bVal, this.buffer2);
    }

    /* Transliterated from the C++ code: */
    public byte getZMCO() {
        byte value = readReg(Register.ZMCO);
        return value;
    }

    public boolean setZPosition(short value) {
        if (value > 0x0FFF) return false;
        writeReg2(Register.ZPOS, value);
        return true;
    }

    public short getZPosition() {
        short value = readReg2(Register.ZPOS, 0xFFF);
        return value;
    }

    public boolean setMPosition(short value) {
        if (value > 0x0FFF) return false;
        writeReg2(Register.MPOS, value);
        return true;
    }

    public short getMPosition() {
        short value = readReg2(Register.MPOS, 0x0FFF);
        return value;
    }

    public boolean setMaxAngle(short value) {
        if (value > 0x0FFF) return false;
        writeReg2(Register.MANG, value);
        return true;
    }

    public short getMaxAngle() {
        short value = readReg2(Register.MANG, 0x0FFF);
        return value;
    }

    public boolean setConfigure(short value) {
        if (value > 0x3FFF) return false;
        writeReg2(Register.CONF, value);
        return true;
    }

    public short getConfigure() {
        short value = readReg2(Register.CONF, 0x3FFF);
        return value;
    }

    //  details configure
    public boolean setPowerMode(byte powerMode) {
        if (powerMode > 3) return false;
        byte value = readReg(Register.CONF_1);
        value &= ~Register.CONF_POWER_MODE.bVal;
        value |= powerMode;
        writeReg(Register.CONF_1, value);
        return true;
    }

    public byte getPowerMode() {
        return readReg(Register.CONF_1, 0x03);
    }

    public boolean setHysteresis(byte hysteresis) {
        if (hysteresis > 3) return false;
        byte value = readReg(Register.CONF_1);
        value &= ~Register.CONF_HYSTERESIS.bVal;
        value |= (hysteresis << 2);
        writeReg(Register.CONF_1, value);
        return true;
    }

    public byte getHysteresis() {
        return (byte) ((readReg(Register.CONF_1) >> 2) & 0x03);
    }

    public boolean setOutputMode(byte outputMode) {
        if (outputMode > 2) return false;
        byte value = readReg(Register.CONF_1);
        value &= ~Register.CONF_OUTPUT_MODE.bVal;
        value |= (outputMode << 4);
        writeReg(Register.CONF_1, value);
        return true;
    }

    public byte getOutputMode() {
        return (byte) ((readReg(Register.CONF_1) >> 4) & 0x03);
    }

    public boolean setPWMFrequency(byte pwmFreq) {
        if (pwmFreq > 3) return false;
        byte value = readReg(Register.CONF_1);
        value &= ~Register.CONF_PWM_FREQUENCY.bVal;
        value |= (pwmFreq << 6);
        writeReg(Register.CONF_1, value);
        return true;
    }

    public byte getPWMFrequency() {
        return (byte) ((readReg(Register.CONF_1) >> 6) & 0x03);
    }

    public boolean setSlowFilter(byte mask) {
        if (mask > 3) return false;
        byte value = readReg(Register.CONF);
        value &= ~Register.CONF_SLOW_FILTER.bVal;
        value |= mask;
        writeReg(Register.CONF, value);
        return true;
    }

    public byte getSlowFilter() {
        return readReg(Register.CONF, 0x03);
    }

    public boolean setFastFilter(byte mask) {
        if (mask > 7) return false;
        byte value = readReg(Register.CONF);
        value &= ~Register.CONF_FAST_FILTER.bVal;
        value |= (mask << 2);
        writeReg(Register.CONF, value);
        return true;
    }

    public byte getFastFilter() {
        return (byte) ((readReg(Register.CONF) >> 2) & 0x07);
    }

    public boolean setWatchDog(byte mask) {
        if (mask > 1) return false;
        byte value = readReg(Register.CONF);
        value &= ~Register.CONF_WATCH_DOG.bVal;
        value |= (mask << 5);
        writeReg(Register.CONF, value);
        return true;
    }

    public byte getWatchDog() {
        return (byte) ((readReg(Register.CONF) >> 5) & 0x01);
    }

    /////////////////////////////////////////////////////////
    //
    //  OUTPUT REGISTERS
    //
    public short rawAngle() {
        short value = readReg2(Register.RAW_ANGLE, 0x0FFF);
        if (this.offset > 0) value = (short) ((value + this.offset) & 0x0FFF);
        // TODO: Allow reversing the direction. Easy in software...
        return value;
    }

    public short readAngle() {
        short value = readReg2(Register.ANGLE, 0x0FFF);
        if (this.offset > 0) value = (short) ((value + this.offset) & 0x0FFF);

        // TODO: Allow reversing the direction. Easy in software...
        return value;
    }

    public boolean setOffset(float degrees) {
        // expect loss of precision.
        if (Math.abs(degrees) > 36000f) return false;
        boolean neg = (degrees < 0);
        if (neg) degrees = -degrees;

        short offs = (short) (Math.round(degrees * (4096 / 360.0f)));
        offs &= 4095;
        if (neg) offs = (short) (4096 - offs);
        this.offset = offs;
        return true;
    }

    public float getOffset() {
        return this.offset * Constants.RAW_TO_DEGREES;
    }

    /////////////////////////////////////////////////////////
    //
    //  STATUS REGISTERS
    //
    public byte readStatus() {
        byte value = readReg(Register.STATUS);
        return value;
    }

    public byte readAGC() {
        byte value = readReg(Register.AGC);
        return value;
    }

    public short readMagnitude() {
        short value = readReg2(Register.MAGNITUDE, 0x0FFF);
        return value;
    }

    public boolean detectMagnet() {
        return (readStatus() & Constants.MAGNET_DETECT) > 1;
    }

    public boolean magnetTooStrong() {
        return (readStatus() & Constants.MAGNET_HIGH) > 1;
    }

    public boolean magnetTooWeak() {
        return (readStatus() & Constants.MAGNET_LOW) > 1;
    }

    public float getAngularSpeed(byte mode) {
        int angle = readAngle();
        double deltaT = this.lastMeasurement.milliseconds();
        this.lastMeasurement.reset();
        int deltaA = angle - this.lastAngle;

        //  assumption is that there is no more than 180° rotation
        //  between two consecutive measurements.
        //  => at least two measurements per rotation (preferred 4).
        if (deltaA > 2048) deltaA -= 4096;
        if (deltaA < -2048) deltaA += 4096;
        float speed = (float) ((deltaA * 1e6) / deltaT);

        //  remember last time & angle
        this.lastAngle = (short) angle;
        //  return degrees or radians
        if (mode == Constants.MODE_RADIANS) {
            return speed * Constants.RAW_TO_RADIANS;
        }
        //  default return degrees
        return speed * Constants.RAW_TO_DEGREES;
    }
}
