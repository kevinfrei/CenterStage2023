package org.firstinspires.ftc.learnbot.devicedrivers.vl53l4cx;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver
 */
public class VL53L4CX extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    // platform_user_config.h
    static final int BYTES_PER_WORD = 2;
    static final int BYTES_PER_DWORD = 4;


    static final int BOOT_COMPLETION_POLLING_TIMEOUT_MS = 500;
    static final int RANGE_COMPLETION_POLLING_TIMEOUT_MS = 2000;
    static final int TEST_COMPLETION_POLLING_TIMEOUT_MS = 60000;

    static final int POLLING_DELAY_MS = 1;


    static final int TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS = 0x8000;
    static final int TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS = 0xC000;

    static final int GAIN_FACTOR__STANDARD_DEFAULT = 0x0800;

    static final int GAIN_FACTOR__HISTOGRAM_DEFAULT = 0x0800;


    static final int OFFSET_CAL_MIN_EFFECTIVE_SPADS = 0x0500;


    static final int OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS = 0x1900;


    static final int OFFSET_CAL_MAX_SIGMA_MM = 0x0040;


    static final int ZONE_CAL_MAX_PRE_PEAK_RATE_MCPS = 0x1900;


    static final int ZONE_CAL_MAX_SIGMA_MM = 0x0040;


    static final int XTALK_EXTRACT_MAX_SIGMA_MM = 0x008C;


    static final int MAX_USER_ZONES = 5;

    static final int MAX_RANGE_RESULTS = 4;


    static final int MAX_STRING_LENGTH = 512;

    public enum Error {
        NONE(0),
        CALIBRATION_WARNING(-1),
        /*!< Warning invalid calibration data may be in used
         *  \a VL53L4CX_InitData()
         *  \a VL53L4CX_GetOffsetCalibrationData
         *  \a VL53L4CX_SetOffsetCalibrationData
         */
        MIN_CLIPPED(-2),
        /*!< Warning parameter passed was clipped to min before to be applied */

        UNDEFINED(-3),
        /*!< Unqualified error */
        INVALID_PARAMS(-4),
        /*!< Parameter passed is invalid or out of range */
        NOT_SUPPORTED(-5),
        /*!< Function is not supported in current mode or configuration */
        RANGE_ERROR(-6),
        /*!< Device report a ranging error interrupt status */
        TIME_OUT(-7),
        /*!< Aborted due to time out */
        MODE_NOT_SUPPORTED(-8),
        /*!< Asked mode is not supported by the device */
        BUFFER_TOO_SMALL(-9),
        /*!< ... */
        COMMS_BUFFER_TOO_SMALL(-10),
        /*!< Supplied buffer is larger than I2C supports */
        GPIO_NOT_EXISTING(-11),
        /*!< User tried to setup a non-existing GPIO pin */
        GPIO_FUNCTIONALITY_NOT_SUPPORTED(-12),
        /*!< unsupported GPIO functionality */
        CONTROL_INTERFACE(-13),
        /*!< error reported from IO functions */
        INVALID_COMMAND(-14),
        /*!< The command is not allowed in the current device state
         *  (power down)
         */
        DIVISION_BY_ZERO(-15),
        /*!< In the function a division by zero occurs */
        REF_SPAD_INIT(-16),
        /*!< Error during reference SPAD initialization */
        GPH_SYNC_CHECK_FAIL(-17),
        /*!<  GPH sync interrupt check fail - API out of sync with device*/
        STREAM_COUNT_CHECK_FAIL(-18),
        /*!<  Stream count check fail - API out of sync with device */
        GPH_ID_CHECK_FAIL(-19),
        /*!<  GPH ID check fail - API out of sync with device */
        ZONE_STREAM_COUNT_CHECK_FAIL(-20),
        /*!<  Zone dynamic config stream count check failed - API out of sync */
        ZONE_GPH_ID_CHECK_FAIL(-21),
        /*!<  Zone dynamic config GPH ID check failed - API out of sync */

        XTALK_EXTRACTION_NO_SAMPLE_FAIL(-22),
        /*!<  Thrown when run_xtalk_extraction fn has 0 successful samples
         * when using the full array to sample the xtalk. In this case there is
         * not enough information to generate new Xtalk param info. The function
         * will exit and leave the current xtalk parameters unaltered
         */
        XTALK_EXTRACTION_SIGMA_LIMIT_FAIL(-23),
        /*!<  Thrown when run_xtalk_extraction fn has found that the
         * avg sigma estimate of the full array xtalk sample is > than the
         * maximal limit allowed. In this case the xtalk sample is too noisy for
         * measurement. The function will exit and leave the current xtalk
         * parameters unaltered.
         */


        OFFSET_CAL_NO_SAMPLE_FAIL(-24),
        /*!<  Thrown if there one of stages has no valid offset calibration
         *    samples. A fatal error calibration not valid
         */
        OFFSET_CAL_NO_SPADS_ENABLED_FAIL(-25),
        /*!<  Thrown if there one of stages has zero effective SPADS
         *    Traps the case when MM1 SPADs is zero.
         *    A fatal error calibration not valid
         */
        ZONE_CAL_NO_SAMPLE_FAIL(-26),
        /*!<  Thrown if then some of the zones have no valid samples
         *    A fatal error calibration not valid
         */

        TUNING_PARM_KEY_MISMATCH(-27),
        /*!<  Thrown if the tuning file key table version does not match with
         * expected value. The driver expects the key table version to match
         * the compiled default version number in the define
         * VL53L4CX_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT
         */

        WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS(-28),
        /*!<  Thrown if there are less than 5 good SPADs are available. */
        WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH(-29),
        /*!<  Thrown if the final reference rate is greater than
         * the upper reference rate limit - default is 40 Mcps.
         * Implies a minimum Q3 (x10) SPAD (5) selected
         */
        WARNING_REF_SPAD_CHAR_RATE_TOO_LOW(-30),
        /*!<  Thrown if the final reference rate is less than
         * the lower reference rate limit - default is 10 Mcps.
         * Implies maximum Q1 (x1) SPADs selected
         */


        WARNING_OFFSET_CAL_MISSING_SAMPLES(-31),
        /*!<  Thrown if there is less than the requested number of
         *    valid samples.
         */
        WARNING_OFFSET_CAL_SIGMA_TOO_HIGH(-32),
        /*!<  Thrown if the offset calibration range sigma estimate is greater
         *    than 8.0 mm. This is the recommended min value to yield a stable
         *    offset measurement
         */
        WARNING_OFFSET_CAL_RATE_TOO_HIGH(-33),
        /*!< Thrown when VL53L4CX_run_offset_calibration()  peak rate is greater
         * than that 50.0Mcps. This is the recommended  max rate to avoid
         * pile-up influencing the offset measurement
         */
        WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW(-34),
        /*!< Thrown when VL53L4CX_run_offset_calibration() when one of stages
         * range has less that 5.0 effective SPADS. This is the recommended
         * min value to yield a stable offset
         */


        WARNING_ZONE_CAL_MISSING_SAMPLES(-35),
        /*!<  Thrown if one of more of the zones have less than
         * the requested number of valid samples
         */
        WARNING_ZONE_CAL_SIGMA_TOO_HIGH(-36),
        /*!<  Thrown if one or more zones have sigma estimate value greater
         *    than 8.0 mm. This is the recommended min value to yield a stable
         *    offset measurement
         */
        WARNING_ZONE_CAL_RATE_TOO_HIGH(-37),
        /*!< Thrown if one of more zones have  peak rate higher than
         * that 50.0Mcps. This is the recommended  max rate to avoid
         * pile-up influencing the offset measurement
         */


        WARNING_XTALK_MISSING_SAMPLES(-38),
        /*!< Thrown to notify that some of the xtalk samples did not yield
         * valid ranging pulse data while attempting to measure
         * the xtalk signal in vl53lx_run_xtalk_extract(). This can signify any
         * of the zones are missing samples, for further debug information the
         * xtalk_results struct should be referred to. This warning is for
         * notification only, xtalk pulse and shape have still been generated
         */
        WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT(-39),
        /*!< Thrown to notify that some of the xtalk samples used for gradient
         * generation did not yield valid ranging pulse data while attempting to
         * measure the xtalk signal in vl53lx_run_xtalk_extract(). This can
         * signify that any one of the zones 0-3 yielded no successful samples.
         * xtalk_results struct should be referred to for further debug info.
         * This warning is for notification only, the xtalk pulse and shape
         * have still been generated.
         */
        WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT(-40),
        /*!< Thrown to notify that some of the xtalk samples used for gradient
         * generation did not pass the sigma limit check  while attempting to
         * measure the xtalk signal in vl53lx_run_xtalk_extract(). This can
         * signify that any one of the zones 0-3 yielded an avg sigma_mm
         * value > the limit. The xtalk_results struct should be referred to for
         * further debug info.
         * This warning is for notification only, the xtalk pulse and shape
         * have still been generated.
         */

        NOT_IMPLEMENTED(-41),
        /*!< Tells requested functionality has not been implemented yet or
         * not compatible with the device
         */
        PLATFORM_SPECIFIC_START(-60);
        /*!< Tells the starting code for platform */
        /**
         * @} VL53L4CX_define_Error_group
         */

        public int bVal;

        Error(int bVal) {
            this.bVal = bVal;
        }
    }

    static final byte I2C =                      0x01;
    static final byte SPI =                     0x00;





    enum WaitMethod {
        BLOCKING(0),
        NON_BLOCKING(1);
        public int bVal;
        WaitMethod(int bVal) {
            this.bVal = bVal;
        }
        }



    enum DeviceState {

        POWERDOWN( 0),

        HW_STANDBY(  1),

        FW_COLDBOOT(  2),

        SW_STANDBY(  3),

        RANGING_DSS_AUTO(  4),

        RANGING_DSS_MANUAL(  5),

        RANGING_WAIT_GPH_SYNC(  6),

        RANGING_GATHER_DATA(  7),

        RANGING_OUTPUT_DATA(  8),

        UNKNOWN( 98),

        ERROR( 99);

        public int bVal;

        DeviceState(int bVal) {
            this.bVal = bVal;
        }
        }

    // typedef uint8_t VL53L4CX_DeviceZonePreset;


    enum DevicePresetModes {
        NONE(0),
        HISTOGRAM_LONG_RANGE                (27),
        HISTOGRAM_MEDIUM_RANGE                (30),
        HISTOGRAM_SHORT_RANGE                (33);
        public byte bVal;

        DevicePresetModes(byte bVal) {
            this.bVal = bVal;
        }
    }




    typedef uint8_t VL53L4CX_DeviceMeasurementModes;

#define VL53L4CX_DEVICEMEASUREMENTMODE_STOP
            ((VL53L4CX_DeviceMeasurementModes)  0x00)
            #define VL53L4CX_DEVICEMEASUREMENTMODE_SINGLESHOT
            ((VL53L4CX_DeviceMeasurementModes)  0x10)
            #define VL53L4CX_DEVICEMEASUREMENTMODE_BACKTOBACK
            ((VL53L4CX_DeviceMeasurementModes)  0x20)
            #define VL53L4CX_DEVICEMEASUREMENTMODE_TIMED
            ((VL53L4CX_DeviceMeasurementModes)  0x40)
            #define VL53L4CX_DEVICEMEASUREMENTMODE_ABORT
            ((VL53L4CX_DeviceMeasurementModes)  0x80)





    typedef uint8_t VL53L4CX_OffsetCalibrationMode;

#define VL53L4CX_OFFSETCALIBRATIONMODE__NONE
            ((VL53L4CX_OffsetCalibrationMode)  0)
            #define VL53L4CX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD
            ((VL53L4CX_OffsetCalibrationMode)  1)
            #define VL53L4CX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM
            ((VL53L4CX_OffsetCalibrationMode)  2)
            #define VL53L4CX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY
            ((VL53L4CX_OffsetCalibrationMode)  3)
            #define VL53L4CX_OFFSETCALIBRATIONMODE__MM1_MM2__HISTOGRAM_PRE_RANGE_ONLY
            ((VL53L4CX_OffsetCalibrationMode)  4)





    typedef uint8_t VL53L4CX_OffsetCorrectionMode;

#define VL53L4CX_OFFSETCORRECTIONMODE__NONE
            ((VL53L4CX_OffsetCorrectionMode)  0)
            #define VL53L4CX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS
            ((VL53L4CX_OffsetCorrectionMode)  1)
            #define VL53L4CX_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS
            ((VL53L4CX_OffsetCorrectionMode)  3)





    typedef uint8_t VL53L4CX_DeviceDmaxMode;

#define VL53L4CX_DEVICEDMAXMODE__NONE
            ((VL53L4CX_DeviceDmaxMode)  0)
            #define VL53L4CX_DEVICEDMAXMODE__FMT_CAL_DATA
            ((VL53L4CX_DeviceDmaxMode)  1)
            #define VL53L4CX_DEVICEDMAXMODE__CUST_CAL_DATA
            ((VL53L4CX_DeviceDmaxMode)  2)





    typedef uint8_t VL53L4CX_DeviceSequenceConfig;

#define VL53L4CX_DEVICESEQUENCECONFIG_VHV
            ((VL53L4CX_DeviceSequenceConfig) 0)
            #define VL53L4CX_DEVICESEQUENCECONFIG_PHASECAL
            ((VL53L4CX_DeviceSequenceConfig) 1)
            #define VL53L4CX_DEVICESEQUENCECONFIG_REFERENCE_PHASE
            ((VL53L4CX_DeviceSequenceConfig) 2)
            #define VL53L4CX_DEVICESEQUENCECONFIG_DSS1
            ((VL53L4CX_DeviceSequenceConfig) 3)
            #define VL53L4CX_DEVICESEQUENCECONFIG_DSS2
            ((VL53L4CX_DeviceSequenceConfig) 4)
            #define VL53L4CX_DEVICESEQUENCECONFIG_MM1
            ((VL53L4CX_DeviceSequenceConfig) 5)
            #define VL53L4CX_DEVICESEQUENCECONFIG_MM2
            ((VL53L4CX_DeviceSequenceConfig) 6)
            #define VL53L4CX_DEVICESEQUENCECONFIG_RANGE
            ((VL53L4CX_DeviceSequenceConfig) 7)





    typedef uint8_t VL53L4CX_DeviceInterruptPolarity;

#define VL53L4CX_DEVICEINTERRUPTPOLARITY_ACTIVE_HIGH
            ((VL53L4CX_DeviceInterruptPolarity)  0x00)
            #define VL53L4CX_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW
            ((VL53L4CX_DeviceInterruptPolarity)  0x10)
            #define VL53L4CX_DEVICEINTERRUPTPOLARITY_BIT_MASK
            ((VL53L4CX_DeviceInterruptPolarity)  0x10)
            #define VL53L4CX_DEVICEINTERRUPTPOLARITY_CLEAR_MASK
            ((VL53L4CX_DeviceInterruptPolarity)  0xEF)





    typedef uint8_t VL53L4CX_DeviceGpioMode;

#define VL53L4CX_DEVICEGPIOMODE_OUTPUT_CONSTANT_ZERO
            ((VL53L4CX_DeviceGpioMode)  0x00)
            #define VL53L4CX_DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS
            ((VL53L4CX_DeviceGpioMode)  0x01)
            #define VL53L4CX_DEVICEGPIOMODE_OUTPUT_TIMIER_INTERRUPTS
            ((VL53L4CX_DeviceGpioMode)  0x02)
            #define VL53L4CX_DEVICEGPIOMODE_OUTPUT_RANGE_MODE_INTERRUPT_STATUS
            ((VL53L4CX_DeviceGpioMode)  0x03)
            #define VL53L4CX_DEVICEGPIOMODE_OUTPUT_SLOW_OSCILLATOR_CLOCK
            ((VL53L4CX_DeviceGpioMode)  0x04)
            #define VL53L4CX_DEVICEGPIOMODE_BIT_MASK
            ((VL53L4CX_DeviceGpioMode)  0x0F)
            #define VL53L4CX_DEVICEGPIOMODE_CLEAR_MASK
            ((VL53L4CX_DeviceGpioMode)  0xF0)





    typedef uint8_t VL53L4CX_DeviceError;

#define VL53L4CX_DEVICEERROR_NOUPDATE
            ((VL53L4CX_DeviceError) 0)

            #define VL53L4CX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE
            ((VL53L4CX_DeviceError) 1)
            #define VL53L4CX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE
            ((VL53L4CX_DeviceError) 2)
            #define VL53L4CX_DEVICEERROR_NOVHVVALUEFOUND
            ((VL53L4CX_DeviceError) 3)
            #define VL53L4CX_DEVICEERROR_MSRCNOTARGET
            ((VL53L4CX_DeviceError) 4)
            #define VL53L4CX_DEVICEERROR_RANGEPHASECHECK
            ((VL53L4CX_DeviceError) 5)
            #define VL53L4CX_DEVICEERROR_SIGMATHRESHOLDCHECK
            ((VL53L4CX_DeviceError) 6)
            #define VL53L4CX_DEVICEERROR_PHASECONSISTENCY
            ((VL53L4CX_DeviceError) 7)
            #define VL53L4CX_DEVICEERROR_MINCLIP
            ((VL53L4CX_DeviceError) 8)
            #define VL53L4CX_DEVICEERROR_RANGECOMPLETE
            ((VL53L4CX_DeviceError) 9)
            #define VL53L4CX_DEVICEERROR_ALGOUNDERFLOW
            ((VL53L4CX_DeviceError) 10)
            #define VL53L4CX_DEVICEERROR_ALGOOVERFLOW
            ((VL53L4CX_DeviceError) 11)
            #define VL53L4CX_DEVICEERROR_RANGEIGNORETHRESHOLD
            ((VL53L4CX_DeviceError) 12)
            #define VL53L4CX_DEVICEERROR_USERROICLIP
            ((VL53L4CX_DeviceError) 13)
            #define VL53L4CX_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS
            ((VL53L4CX_DeviceError) 14)
            #define VL53L4CX_DEVICEERROR_REFSPADCHARMORETHANTARGET
            ((VL53L4CX_DeviceError) 15)
            #define VL53L4CX_DEVICEERROR_REFSPADCHARLESSTHANTARGET
            ((VL53L4CX_DeviceError) 16)
            #define VL53L4CX_DEVICEERROR_MULTCLIPFAIL
            ((VL53L4CX_DeviceError) 17)
            #define VL53L4CX_DEVICEERROR_GPHSTREAMCOUNT0READY
            ((VL53L4CX_DeviceError) 18)
            #define VL53L4CX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK
            ((VL53L4CX_DeviceError) 19)
            #define VL53L4CX_DEVICEERROR_EVENTCONSISTENCY
            ((VL53L4CX_DeviceError) 20)
            #define VL53L4CX_DEVICEERROR_MINSIGNALEVENTCHECK
            ((VL53L4CX_DeviceError) 21)
            #define VL53L4CX_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE
            ((VL53L4CX_DeviceError) 22)


            #define VL53L4CX_DEVICEERROR_PREV_RANGE_NO_TARGETS
            ((VL53L4CX_DeviceError) 23)





    typedef uint8_t VL53L4CX_DeviceReportStatus;

#define VL53L4CX_DEVICEREPORTSTATUS_NOUPDATE
            ((VL53L4CX_DeviceReportStatus) 0)

            #define VL53L4CX_DEVICEREPORTSTATUS_ROI_SETUP
            ((VL53L4CX_DeviceReportStatus)  1)
            #define VL53L4CX_DEVICEREPORTSTATUS_VHV
            ((VL53L4CX_DeviceReportStatus)  2)
            #define VL53L4CX_DEVICEREPORTSTATUS_PHASECAL
            ((VL53L4CX_DeviceReportStatus)  3)
            #define VL53L4CX_DEVICEREPORTSTATUS_REFERENCE_PHASE
            ((VL53L4CX_DeviceReportStatus)  4)
            #define VL53L4CX_DEVICEREPORTSTATUS_DSS1
            ((VL53L4CX_DeviceReportStatus)  5)
            #define VL53L4CX_DEVICEREPORTSTATUS_DSS2
            ((VL53L4CX_DeviceReportStatus)  6)
            #define VL53L4CX_DEVICEREPORTSTATUS_MM1
            ((VL53L4CX_DeviceReportStatus)  7)
            #define VL53L4CX_DEVICEREPORTSTATUS_MM2
            ((VL53L4CX_DeviceReportStatus)  8)
            #define VL53L4CX_DEVICEREPORTSTATUS_RANGE
            ((VL53L4CX_DeviceReportStatus)  9)
            #define VL53L4CX_DEVICEREPORTSTATUS_HISTOGRAM
            ((VL53L4CX_DeviceReportStatus) 10)





    typedef uint8_t VL53L4CX_DeviceDssMode;

#define VL53L4CX_DEVICEDSSMODE__DISABLED
            ((VL53L4CX_DeviceDssMode) 0)
            #define VL53L4CX_DEVICEDSSMODE__TARGET_RATE
            ((VL53L4CX_DeviceDssMode) 1)
            #define VL53L4CX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS
            ((VL53L4CX_DeviceDssMode) 2)
            #define VL53L4CX_DEVICEDSSMODE__BLOCK_SELECT
            ((VL53L4CX_DeviceDssMode) 3)






    typedef uint8_t VL53L4CX_HistAlgoSelect;

#define VL53L4CX_HIST_ALGO_SELECT__PW_HIST_GEN1
            ((VL53L4CX_HistAlgoSelect) 1)
            #define VL53L4CX_HIST_ALGO_SELECT__PW_HIST_GEN2
            ((VL53L4CX_HistAlgoSelect) 2)
            #define VL53L4CX_HIST_ALGO_SELECT__PW_HIST_GEN3
            ((VL53L4CX_HistAlgoSelect) 3)
            #define VL53L4CX_HIST_ALGO_SELECT__PW_HIST_GEN4
            ((VL53L4CX_HistAlgoSelect) 4)






    typedef uint8_t VL53L4CX_HistTargetOrder;

#define VL53L4CX_HIST_TARGET_ORDER__INCREASING_DISTANCE
            ((VL53L4CX_HistTargetOrder) 1)
            #define VL53L4CX_HIST_TARGET_ORDER__STRONGEST_FIRST
            ((VL53L4CX_HistTargetOrder) 2)






    typedef uint8_t VL53L4CX_HistAmbEstMethod;

#define VL53L4CX_HIST_AMB_EST_METHOD__AMBIENT_BINS
            ((VL53L4CX_HistAmbEstMethod) 1)
            #define VL53L4CX_HIST_AMB_EST_METHOD__THRESHOLDED_BINS
            ((VL53L4CX_HistAmbEstMethod) 2)






    typedef uint8_t VL53L4CX_HistXtalkCompEnable;

#define VL53L4CX_HIST_XTALK_COMP__DIS
            ((VL53L4CX_HistXtalkCompEnable) 0)
            #define VL53L4CX_HIST_XTALK_COMP__EN
            ((VL53L4CX_HistXtalkCompEnable) 1)




    typedef uint8_t VL53L4CX_DeviceConfigLevel;

#define VL53L4CX_DEVICECONFIGLEVEL_SYSTEM_CONTROL
            ((VL53L4CX_DeviceConfigLevel)  0)

            #define VL53L4CX_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS
            ((VL53L4CX_DeviceConfigLevel)  1)

            #define VL53L4CX_DEVICECONFIGLEVEL_TIMING_ONWARDS
            ((VL53L4CX_DeviceConfigLevel)  2)

            #define VL53L4CX_DEVICECONFIGLEVEL_GENERAL_ONWARDS
            ((VL53L4CX_DeviceConfigLevel)  3)

            #define VL53L4CX_DEVICECONFIGLEVEL_STATIC_ONWARDS
            ((VL53L4CX_DeviceConfigLevel)  4)

            #define VL53L4CX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS
            ((VL53L4CX_DeviceConfigLevel)  5)

            #define VL53L4CX_DEVICECONFIGLEVEL_FULL
            ((VL53L4CX_DeviceConfigLevel)  6)






    typedef uint8_t VL53L4CX_DeviceResultsLevel;

#define VL53L4CX_DEVICERESULTSLEVEL_SYSTEM_RESULTS
            ((VL53L4CX_DeviceResultsLevel)  0)

            #define VL53L4CX_DEVICERESULTSLEVEL_UPTO_CORE
            ((VL53L4CX_DeviceResultsLevel)  1)

            #define VL53L4CX_DEVICERESULTSLEVEL_FULL
            ((VL53L4CX_DeviceResultsLevel)  2)







    typedef uint8_t VL53L4CX_DeviceTestMode;

#define VL53L4CX_DEVICETESTMODE_NONE
            ((VL53L4CX_DeviceTestMode) 0x00)

            #define VL53L4CX_DEVICETESTMODE_NVM_ZERO
            ((VL53L4CX_DeviceTestMode) 0x01)

            #define VL53L4CX_DEVICETESTMODE_NVM_COPY
            ((VL53L4CX_DeviceTestMode) 0x02)

            #define VL53L4CX_DEVICETESTMODE_PATCH
            ((VL53L4CX_DeviceTestMode) 0x03)

            #define VL53L4CX_DEVICETESTMODE_DCR
            ((VL53L4CX_DeviceTestMode) 0x04)

            #define VL53L4CX_DEVICETESTMODE_LCR_VCSEL_OFF
            ((VL53L4CX_DeviceTestMode) 0x05)

            #define VL53L4CX_DEVICETESTMODE_LCR_VCSEL_ON
            ((VL53L4CX_DeviceTestMode) 0x06)

            #define VL53L4CX_DEVICETESTMODE_SPOT_CENTRE_LOCATE
            ((VL53L4CX_DeviceTestMode) 0x07)

            #define VL53L4CX_DEVICETESTMODE_REF_SPAD_CHAR_WITH_PRE_VHV
            ((VL53L4CX_DeviceTestMode) 0x08)

            #define VL53L4CX_DEVICETESTMODE_REF_SPAD_CHAR_ONLY
            ((VL53L4CX_DeviceTestMode) 0x09)







    typedef uint8_t VL53L4CX_DeviceSscArray;

#define VL53L4CX_DEVICESSCARRAY_RTN ((VL53L4CX_DeviceSscArray) 0x00)

            #define VL53L4CX_DEVICETESTMODE_REF ((VL53L4CX_DeviceSscArray) 0x01)







            #define VL53L4CX_RETURN_ARRAY_ONLY                   0x01

            #define VL53L4CX_REFERENCE_ARRAY_ONLY                0x10

            #define VL53L4CX_BOTH_RETURN_AND_REFERENCE_ARRAYS    0x11

            #define VL53L4CX_NEITHER_RETURN_AND_REFERENCE_ARRAYS 0x00






            #define VL53L4CX_DEVICEINTERRUPTLEVEL_ACTIVE_HIGH               0x00

            #define VL53L4CX_DEVICEINTERRUPTLEVEL_ACTIVE_LOW                0x10

            #define VL53L4CX_DEVICEINTERRUPTLEVEL_ACTIVE_MASK               0x10






            #define VL53L4CX_POLLING_DELAY_US                     1000

            #define VL53L4CX_SOFTWARE_RESET_DURATION_US            100

            #define VL53L4CX_FIRMWARE_BOOT_TIME_US                1200

            #define VL53L4CX_ENABLE_POWERFORCE_SETTLING_TIME_US    250

            #define VL53L4CX_SPAD_ARRAY_WIDTH                       16

            #define VL53L4CX_SPAD_ARRAY_HEIGHT                      16

            #define VL53L4CX_NVM_SIZE_IN_BYTES                     512

            #define VL53L4CX_NO_OF_SPAD_ENABLES                    256

            #define VL53L4CX_RTN_SPAD_BUFFER_SIZE                   32

            #define VL53L4CX_REF_SPAD_BUFFER_SIZE                    6

            #define VL53L4CX_AMBIENT_WINDOW_VCSEL_PERIODS          256

            #define VL53L4CX_RANGING_WINDOW_VCSEL_PERIODS         2048

            #define VL53L4CX_MACRO_PERIOD_VCSEL_PERIODS
            (VL53L4CX_AMBIENT_WINDOW_VCSEL_PERIODS +
    VL53L4CX_RANGING_WINDOW_VCSEL_PERIODS)

            #define VL53L4CX_MAX_ALLOWED_PHASE                    0xFFFF


            #define VL53L4CX_RTN_SPAD_UNITY_TRANSMISSION      0x0100

            #define VL53L4CX_RTN_SPAD_APERTURE_TRANSMISSION   0x0038


            #define VL53L4CX_SPAD_TOTAL_COUNT_MAX                 ((0x01 << 29) - 1)

            #define VL53L4CX_SPAD_TOTAL_COUNT_RES_THRES            (0x01 << 24)

#define VL53L4CX_COUNT_RATE_INTERNAL_MAX              ((0x01 << 24) - 1)

            #define VL53L4CX_SPEED_OF_LIGHT_IN_AIR                299704

            #define VL53L4CX_SPEED_OF_LIGHT_IN_AIR_DIV_8          (299704 >> 3)








    typedef uint8_t VL53L4CX_ZoneConfig_BinConfig_select;

#define VL53L4CX_ZONECONFIG_BINCONFIG__LOWAMB
            ((VL53L4CX_ZoneConfig_BinConfig_select) 1)
            #define VL53L4CX_ZONECONFIG_BINCONFIG__MIDAMB
            ((VL53L4CX_ZoneConfig_BinConfig_select) 2)
            #define VL53L4CX_ZONECONFIG_BINCONFIG__HIGHAMB
            ((VL53L4CX_ZoneConfig_BinConfig_select) 3)





    typedef uint8_t VL53L4CX_GPIO_Interrupt_Mode;

#define VL53L4CX_GPIOINTMODE_LEVEL_LOW
            ((VL53L4CX_GPIO_Interrupt_Mode) 0)

            #define VL53L4CX_GPIOINTMODE_LEVEL_HIGH
            ((VL53L4CX_GPIO_Interrupt_Mode) 1)

            #define VL53L4CX_GPIOINTMODE_OUT_OF_WINDOW
            ((VL53L4CX_GPIO_Interrupt_Mode) 2)

            #define VL53L4CX_GPIOINTMODE_IN_WINDOW
            ((VL53L4CX_GPIO_Interrupt_Mode) 3)






    typedef uint16_t VL53L4CX_TuningParms;

#define VL53L4CX_TUNINGPARMS_LLD_PUBLIC_MIN_ADDRESS
            ((VL53L4CX_TuningParms) VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS)
            #define VL53L4CX_TUNINGPARMS_LLD_PUBLIC_MAX_ADDRESS
            ((VL53L4CX_TuningParms) VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEB)

            #define VL53L4CX_TUNINGPARMS_LLD_PRIVATE_MIN_ADDRESS
            ((VL53L4CX_TuningParms) VL53L4CX_TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS)
            #define VL53L4CX_TUNINGPARMS_LLD_PRIVATE_MAX_ADDRESS
            ((VL53L4CX_TuningParms) VL53L4CX_TUNINGPARMS_LLD_PRIVATE_MIN_ADDRESS)

            #define VL53L4CX_TUNINGPARM_VERSION
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 0))
            #define VL53L4CX_TUNINGPARM_KEY_TABLE_VERSION
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 1))
            #define VL53L4CX_TUNINGPARM_LLD_VERSION
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 2))
            #define VL53L4CX_TUNINGPARM_HIST_ALGO_SELECT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 3))
            #define VL53L4CX_TUNINGPARM_HIST_TARGET_ORDER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 4))
            #define VL53L4CX_TUNINGPARM_HIST_FILTER_WOI_0
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 5))
            #define VL53L4CX_TUNINGPARM_HIST_FILTER_WOI_1
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 6))
            #define VL53L4CX_TUNINGPARM_HIST_AMB_EST_METHOD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 7))
            #define VL53L4CX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 8))
            #define VL53L4CX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 9))
            #define VL53L4CX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 10))
            #define VL53L4CX_TUNINGPARM_HIST_AMB_EVENTS_SCALER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 11))
            #define VL53L4CX_TUNINGPARM_HIST_NOISE_THRESHOLD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 12))
            #define VL53L4CX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 13))
            #define VL53L4CX_TUNINGPARM_HIST_SIGMA_EST_REF_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 14))
            #define VL53L4CX_TUNINGPARM_HIST_SIGMA_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 15))
            #define VL53L4CX_TUNINGPARM_HIST_GAIN_FACTOR
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 16))
            #define VL53L4CX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 17))
            #define VL53L4CX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 18))
            #define VL53L4CX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 19))
            #define VL53L4CX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 20))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 21))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 22))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 23))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 24))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 25))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 26))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 27))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 28))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 29))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 30))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 31))
            #define VL53L4CX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 32))
            #define VL53L4CX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 33))
            #define VL53L4CX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 34))
            #define VL53L4CX_TUNINGPARM_PHASECAL_TARGET
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 35))
            #define VL53L4CX_TUNINGPARM_LITE_CAL_REPEAT_RATE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 36))
            #define VL53L4CX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 37))
            #define VL53L4CX_TUNINGPARM_LITE_MIN_CLIP_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 38))
            #define VL53L4CX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 39))
            #define VL53L4CX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 40))
            #define VL53L4CX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 41))
            #define VL53L4CX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 42))
            #define VL53L4CX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 43))
            #define VL53L4CX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 44))
            #define VL53L4CX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 45))
            #define VL53L4CX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 46))
            #define VL53L4CX_TUNINGPARM_LITE_SIGMA_REF_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 47))
            #define VL53L4CX_TUNINGPARM_LITE_RIT_MULT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 48))
            #define VL53L4CX_TUNINGPARM_LITE_SEED_CONFIG
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 49))
            #define VL53L4CX_TUNINGPARM_LITE_QUANTIFIER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 50))
            #define VL53L4CX_TUNINGPARM_LITE_FIRST_ORDER_SELECT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 51))
            #define VL53L4CX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 52))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 53))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 54))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 55))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 56))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 57))
            #define VL53L4CX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 58))
            #define VL53L4CX_TUNINGPARM_TIMED_SEED_CONFIG
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 59))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 60))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 61))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 62))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 63))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 64))
            #define VL53L4CX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 65))
            #define VL53L4CX_TUNINGPARM_VHV_LOOPBOUND
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 66))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 67))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 68))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 69))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 70))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 71))
            #define VL53L4CX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 72))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 73))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 74))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 75))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 76))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 77))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 78))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 79))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 80))
            #define VL53L4CX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 81))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 82))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 83))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 84))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 85))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 86))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 87))
            #define VL53L4CX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 88))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 89))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 90))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 91))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 92))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 93))
            #define VL53L4CX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 94))
            #define VL53L4CX_TUNINGPARM_SPADMAP_VCSEL_PERIOD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 95))
            #define VL53L4CX_TUNINGPARM_SPADMAP_VCSEL_START
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 96))
            #define VL53L4CX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 97))
            #define VL53L4CX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 98))
            #define VL53L4CX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 99))
            #define VL53L4CX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 100))
            #define VL53L4CX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 101))
            #define VL53L4CX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 102))
            #define VL53L4CX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 103))
            #define VL53L4CX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 104))
            #define VL53L4CX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 105))
            #define VL53L4CX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 106))
            #define VL53L4CX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 107))
            #define VL53L4CX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 108))
            #define VL53L4CX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 109))
            #define VL53L4CX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 110))
            #define VL53L4CX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 111))
            #define VL53L4CX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 112))
            #define VL53L4CX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 113))
            #define VL53L4CX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 114))
            #define VL53L4CX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 115))
            #define VL53L4CX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 116))
            #define VL53L4CX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 117))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 118))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_NOISE_MARGIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 119))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 120))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 121))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 122))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 123))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 124))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_CLIP_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 125))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_SCALER_CALC_METHOD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 126))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 127))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 128))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_USER_SCALER_SET
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 129))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 130))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 131))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 132))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 133))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 134))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 135))
            #define VL53L4CX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 136))
            #define VL53L4CX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 137))
            #define VL53L4CX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 138))
            #define VL53L4CX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 139))
            #define VL53L4CX_TUNINGPARM_PHASECAL_PATCH_POWER
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 140))
            #define VL53L4CX_TUNINGPARM_HIST_MERGE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 141))
            #define VL53L4CX_TUNINGPARM_RESET_MERGE_THRESHOLD
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 142))
            #define VL53L4CX_TUNINGPARM_HIST_MERGE_MAX_SIZE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 143))
            #define VL53L4CX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 144))
            #define VL53L4CX_TUNINGPARM_UWR_ENABLE
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 145))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 146))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 147))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 148))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 149))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 150))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 151))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 152))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 153))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 154))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 155))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 156))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 157))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 158))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 159))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 160))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 161))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 162))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 163))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 164))
            #define VL53L4CX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 165))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_1_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 166))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_1_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 167))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_2_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 168))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_2_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 169))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_3_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 170))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_3_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 171))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_4_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 172))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_4_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 173))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_5_MIN
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 174))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_ZONE_5_MAX
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 175))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 176))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 177))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 178))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 179))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 180))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 181))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 182))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 183))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEA
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 184))
            #define VL53L4CX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEB
            ((VL53L4CX_TuningParms) (VL53L4CX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 185))




            #endif






    class LLDriverData_t {

        byte   wait_method;

        DevicePresetModes        preset_mode;

        DeviceZonePreset         zone_preset;

        DeviceMeasurementModes   measurement_mode;

        OffsetCalibrationMode    offset_calibration_mode;

        OffsetCorrectionMode     offset_correction_mode;

        DeviceDmaxMode           dmax_mode;

        int  phasecal_config_timeout_us;

        int  mm_config_timeout_us;

        int  range_config_timeout_us;

        int  inter_measurement_period_ms;

        short  dss_config__target_total_rate_mcps;

        int  fw_ready_poll_duration_ms;

        byte   fw_ready;

        byte   debug_mode;

        ll_version_t                 version;


        ll_driver_state_t            ll_state;


        GPIO_interrupt_config_t      gpio_interrupt_config;


        customer_nvm_managed_t       customer;
        cal_peak_rate_map_t          cal_peak_rate_map;
        additional_offset_cal_data_t add_off_cal_data;
        dmax_calibration_data_t      fmt_dmax_cal;
        dmax_calibration_data_t      cust_dmax_cal;
        gain_calibration_data_t      gain_cal;
        user_zone_t                  mm_roi;
        optical_centre_t             optical_centre;
        zone_config_t                zone_cfg;


        tuning_parm_storage_t        tuning_parms;


        byte[] rtn_good_spads = new byte[RTN_SPAD_BUFFER_SIZE];


        refspadchar_config_t         refspadchar;
        ssc_config_t                 ssc_cfg;
        hist_post_process_config_t   histpostprocess;
        hist_gen3_dmax_config_t      dmax_cfg;
        xtalkextract_config_t        xtalk_extract_cfg;
        xtalk_config_t               xtalk_cfg;
        offsetcal_config_t           offsetcal_cfg;
        zonecal_config_t             zonecal_cfg;


        static_nvm_managed_t         stat_nvm;
        histogram_config_t           hist_cfg;
        static_config_t              stat_cfg;
        general_config_t             gen_cfg;
        timing_config_t              tim_cfg;
        dynamic_config_t             dyn_cfg;
        system_control_t             sys_ctrl;
        system_results_t             sys_results;
        nvm_copy_data_t              nvm_copy_data;


        histogram_bin_data_t         hist_data;
        histogram_bin_data_t         hist_xtalk;


        xtalk_histogram_data_t       xtalk_shapes;
        xtalk_range_results_t        xtalk_results;
        xtalk_calibration_results_t  xtalk_cal;
        hist_xtalk_extract_data_t    xtalk_extract;


        offset_range_results_t       offset_results;


        core_results_t               core_results;
        debug_results_t              dbg_results;

        smudge_corrector_config_t  smudge_correct_config;

        smudge_corrector_internals_t smudge_corrector_internals;




        low_power_auto_data_t    low_power_auto_data;

        byte[]  wArea1 = new byte[1536];
        byte[]  wArea2 = new byte[512];
        per_vcsel_period_offset_cal_data_t per_vcsel_cal_data;

        byte bin_rec_pos;

        byte pos_before_next_recom;

        int  multi_bins_rec[BIN_REC_SIZE][TIMING_CONF_A_B_SIZE][HISTOGRAM_BUFFER_SIZE];

        short[] PreviousRangeMilliMeter = new short[MAX_RANGE_RESULTS];
        byte[] PreviousRangeStatus = new byte[MAX_RANGE_RESULTS];
        byte[] PreviousExtendedRange = new byte[MAX_RANGE_RESULTS];
        byte PreviousRangeActiveResults;
        byte PreviousStreamCount;
    };

    class DevData_t {
        LLDriverData_t   LLData;
        /*!< Low Level Driver data structure */

        LLDriverResults_t llresults;
        /*!< Low Level Driver data structure */

        DeviceParameters_t CurrentParameters;
        /*!< Current Device Parameter */

    };

    class Dev_t {
        DevData_t   Data;
        /*!< Low Level Driver data structure */
        byte   i2c_slave_address;
        byte   comms_type;
        short  comms_speed_khz;
        byte   I2cDevAddr;
        int     Present;
        int   Enabled;
        int LoopState;
        int FirstStreamCountZero;
        int   Idle;
        int   Ready;
        byte RangeStatus;
        int /* FixPoint1616_t */ SignalRateRtnMegaCps;
        DeviceState   device_state;  /*!< Device State */
    };

    protected VL53L4CX(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    // api_core.cpp

    Error data_init(
            DEV        Dev,
            byte           read_p2p_data)
    {


        Error status       = Error.NONE;
        LLDriverData_t    *pdev =                VL53L4CXDevStructGetLLDriverHandle(Dev);
        LLDriverResults_t *pres =                VL53L4CXDevStructGetLLResultsHandle(Dev);


        zone_objects_t    *pobjects;

        byte i = 0;

        VL53L4CX_init_ll_driver_state(
                Dev,
                DEVICESTATE_UNKNOWN);

        pres->range_results.max_results    = MAX_RANGE_RESULTS;
        pres->range_results.active_results = 0;
        pres->zone_results.max_zones       = MAX_USER_ZONES;
        pres->zone_results.active_zones    = 0;

        for (i = 0; i < MAX_USER_ZONES; i++) {
            pobjects = &(pres->zone_results.VL53L4CX_p_003[i]);
            pobjects->xmonitor.VL53L4CX_p_016 = 0;
            pobjects->xmonitor.VL53L4CX_p_017  = 0;
            pobjects->xmonitor.VL53L4CX_p_011          = 0;
            pobjects->xmonitor.range_status =
                    DEVICEERROR_NOUPDATE;
        }



        pres->zone_hists.max_zones         = MAX_USER_ZONES;
        pres->zone_hists.active_zones      = 0;



        pres->zone_cal.max_zones           = MAX_USER_ZONES;
        pres->zone_cal.active_zones        = 0;
        for (i = 0; i < MAX_USER_ZONES; i++) {
            pres->zone_cal.VL53L4CX_p_003[i].no_of_samples   = 0;
            pres->zone_cal.VL53L4CX_p_003[i].effective_spads = 0;
            pres->zone_cal.VL53L4CX_p_003[i].peak_rate_mcps  = 0;
            pres->zone_cal.VL53L4CX_p_003[i].median_range_mm = 0;
            pres->zone_cal.VL53L4CX_p_003[i].range_mm_offset = 0;
        }

        pdev->wait_method             = WAIT_METHOD_BLOCKING;
        pdev->preset_mode   = DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE;
        pdev->zone_preset             = 0;
        pdev->measurement_mode        = DEVICEMEASUREMENTMODE_STOP;

        pdev->offset_calibration_mode =
                OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
        pdev->offset_correction_mode  =
                OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
        pdev->dmax_mode  =
                DEVICEDMAXMODE__FMT_CAL_DATA;

        pdev->phasecal_config_timeout_us  =  1000;
        pdev->mm_config_timeout_us        =  2000;
        pdev->range_config_timeout_us     = 13000;
        pdev->inter_measurement_period_ms =   100;
        pdev->dss_config__target_total_rate_mcps = 0x0A00;
        pdev->debug_mode                  =  0x00;

        pdev->offset_results.max_results    = MAX_OFFSET_RANGE_RESULTS;
        pdev->offset_results.active_results = 0;



        pdev->gain_cal.standard_ranging_gain_factor =
                TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;
        pdev->gain_cal.histogram_ranging_gain_factor =
                TUNINGPARM_HIST_GAIN_FACTOR_DEFAULT;


        VL53L4CX_init_version(Dev);


        memset(pdev->multi_bins_rec, 0, sizeof(pdev->multi_bins_rec));
        pdev->bin_rec_pos = 0;
        pdev->pos_before_next_recom = 0;



        if (read_p2p_data > 0 && status == Error.NONE) {
            status = VL53L4CX_read_p2p_data(Dev);
        }


        if (status == Error.NONE)
            status = VL53L4CX_init_refspadchar_config_struct(
                    &(pdev->refspadchar));


        if (status == Error.NONE)
            status = VL53L4CX_init_ssc_config_struct(
                    &(pdev->ssc_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_xtalk_config_struct(
                    &(pdev->customer),
               &(pdev->xtalk_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_xtalk_extract_config_struct(
                    &(pdev->xtalk_extract_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_offset_cal_config_struct(
                    &(pdev->offsetcal_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_zone_cal_config_struct(
                    &(pdev->zonecal_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_hist_post_process_config_struct(
                    pdev->xtalk_cfg.global_crosstalk_compensation_enable,
                    &(pdev->histpostprocess));


        if (status == Error.NONE)
            status = VL53L4CX_init_hist_gen3_dmax_config_struct(
                    &(pdev->dmax_cfg));


        if (status == Error.NONE)
            status = VL53L4CX_init_tuning_parm_storage_struct(
                    &(pdev->tuning_parms));



        if (status == Error.NONE)
            status = VL53L4CX_set_preset_mode(
                    Dev,
                    pdev->preset_mode,
                    pdev->dss_config__target_total_rate_mcps,
                    pdev->phasecal_config_timeout_us,
                    pdev->mm_config_timeout_us,
                    pdev->range_config_timeout_us,
                    pdev->inter_measurement_period_ms);


        VL53L4CX_init_histogram_bin_data_struct(
                0,
                HISTOGRAM_BUFFER_SIZE,
                &(pdev->hist_data));

        VL53L4CX_init_histogram_bin_data_struct(
                0,
                HISTOGRAM_BUFFER_SIZE,
                &(pdev->hist_xtalk));


        VL53L4CX_init_xtalk_bin_data_struct(
                0,
                XTALK_HISTO_BINS,
                &(pdev->xtalk_shapes.xtalk_shape));



        VL53L4CX_xtalk_cal_data_init(
                Dev
        );



        VL53L4CX_dynamic_xtalk_correction_data_init(
                Dev
        );



        VL53L4CX_low_power_auto_data_init(
                Dev
        );

        return status;
    }



    // api.cpp
    Error InitSensor(byte address) {
        Error status = Error.NONE;
        // VL53L4CX_Off();
        // VL53L4CX_On();
        status = this.SetDeviceAddress(address);

        if (status == Error.NONE) {
            status = this.WaitDeviceBooted();
        }

        if (status == Error.NONE) {
            status = this.DataInit();
        }

        return status;
    }

    Error WaitDeviceBooted() {
        Error Status = Error.NONE;

        Status = this.poll_for_boot_completion(Dev, BOOT_COMPLETION_POLLING_TIMEOUT_MS);

        return Status;
    }

    Error DataInit() {
        Error Status = Error.NONE;
        LLDriverData_t /* *pdev */ pdev;
        byte measurement_mode;

        //#ifdef USE_I2C_2V8
        //    Status = this.RdByte(Dev, VL53L4CX_PAD_I2C_HV__EXTSUP_CONFIG, /*&*/ i);
        //    if (Status == Error.NONE) {
        //        i = (i & 0xfe) | 0x01;
        //        Status = this.WrByte(Dev, VL53L4CX_PAD_I2C_HV__EXTSUP_CONFIG, i);
        //    }
        //#endif
        if (Status == Error.NONE) {
            Status = this.data_init(Dev, 1);
        }

        if (Status == Error.NONE)
            Status = SetPresetModeL3CX(Dev, DISTANCEMODE_MEDIUM, 1000);


        if (Status == Error.NONE) {
            Status = this.SetMeasurementTimingBudgetMicroSeconds(33333);
        }

        if (Status == Error.NONE) {
            pdev = VL53L4CXDevStructGetLLDriverHandle(Dev);
            memset( & pdev -> per_vcsel_cal_data, 0, sizeof(pdev -> per_vcsel_cal_data));
        }

        if (Status == Error.NONE) {
            Status = this.set_dmax_mode(Dev, DEVICEDMAXMODE__CUST_CAL_DATA);
        }


        if (Status == Error.NONE) {
            Status = this.SmudgeCorrectionEnable(SMUDGE_CORRECTION_NONE);
        }

        measurement_mode = DEVICEMEASUREMENTMODE_BACKTOBACK;
        VL53L4CXDevDataSet(Dev, LLData.measurement_mode, measurement_mode);

        VL53L4CXDevDataSet(Dev, CurrentParameters.DistanceMode, DISTANCEMODE_MEDIUM);

        return Status;
    }

    Error SetDeviceAddress(byte DeviceAddress) {
        Error Status = Error.NONE;
        LLDriverData_t * pdev = VL53L4CXDevStructGetLLDriverHandle(Dev);
        static_nvm_managed_t * pdata = &(pdev -> stat_nvm);

        Status = this.WrByte(Dev, I2C_SLAVE__DEVICE_ADDRESS, DeviceAddress / 2);

        if (Status == Error.NONE) {
            Dev -> I2cDevAddr = DeviceAddress;
        }

        pdata -> i2c_slave__device_address = (DeviceAddress / 2) & 0x7F;

        return Status;
    }

    // class.cpp
    Error WaitValueMaskEx(Dev_t pdev, int timeout_ms, short index, byte value, byte mask, int poll_delay_ms) {

        /*
         * Platform implementation of WaitValueMaskEx V2WReg script command
         *
         * WaitValueMaskEx(
         *          duration_ms,
         *          index,
         *          value,
         *          mask,
         *          poll_delay_ms);
         */

        Error status = Error.NONE;
        ElapsedTime start_time_ms = new ElapsedTime();
        double current_time_ms = 0;
        double polling_time_ms = 0;
        byte byte_value = 0;
        byte found = 0;



        /* calculate time limit in absolute time */

        start_time_ms.reset();

        /* remember current trace functions and temporarily disable
         * function logging
         */


        /* wait until value is found, timeout reached on error occurred */

        while ((status == Error.NONE) &&
                (polling_time_ms < timeout_ms) &&
                (found == 0)) {

            if (status == Error.NONE)
                status = this.RdByte(pdev, index, byte_value);

            if ((byte_value & mask) == value) {
                found = 1;
            }

            if (status == Error.NONE &&
                    found == 0 &&
                    poll_delay_ms > 0)
                status = this.WaitMs(pdev, poll_delay_ms);

    /* Update polling time (Compare difference rather than absolute to
    negate 32bit wrap around issue) */
            current_time_ms = start_time_ms.milliseconds();
            polling_time_ms = current_time_ms;

        }


        if (found == 0 && status == Error.NONE) {
            status = Error.TIME_OUT;
        }

        return status;
    }

    // wait.cpp
    Error poll_for_boot_completion(DEV Dev, int timeout_ms) {


        Error status = Error.NONE;

        status = this.WaitUs(Dev, FIRMWARE_BOOT_TIME_US);

        if (status == Error.NONE)
            status =
                    this.WaitValueMaskEx(
                            Dev,
                            timeout_ms,
                            FIRMWARE__SYSTEM_STATUS,
                            0x01,
                            0x01,
                            POLLING_DELAY_MS);

        if (status == Error.NONE) {
            this.init_ll_driver_state(Dev, DEVICESTATE_SW_STANDBY);
        }

        return status;
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
