


/**
 * Baudrate for the GPS 1 Serial Port
 *
 * Configure the Baudrate for the GPS 1 Serial Port.
 *
 * Note: certain drivers such as the GPS can determine the Baudrate automatically.
 *
 * @value 0 Auto
 * @value 50 50 8N1
 * @value 75 75 8N1
 * @value 110 110 8N1
 * @value 134 134 8N1
 * @value 150 150 8N1
 * @value 200 200 8N1
 * @value 300 300 8N1
 * @value 600 600 8N1
 * @value 1200 1200 8N1
 * @value 1800 1800 8N1
 * @value 2400 2400 8N1
 * @value 4800 4800 8N1
 * @value 9600 9600 8N1
 * @value 19200 19200 8N1
 * @value 38400 38400 8N1
 * @value 57600 57600 8N1
 * @value 115200 115200 8N1
 * @value 230400 230400 8N1
 * @value 460800 460800 8N1
 * @value 500000 500000 8N1
 * @value 921600 921600 8N1
 * @value 1000000 1000000 8N1
 * @value 1500000 1500000 8N1
 * @value 2000000 2000000 8N1
 * @value 3000000 3000000 8N1
 * @group Serial
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SER_GPS1_BAUD, 0);

/**
 * Baudrate for the TELEM 1 Serial Port
 *
 * Configure the Baudrate for the TELEM 1 Serial Port.
 *
 * Note: certain drivers such as the GPS can determine the Baudrate automatically.
 *
 * @value 0 Auto
 * @value 50 50 8N1
 * @value 75 75 8N1
 * @value 110 110 8N1
 * @value 134 134 8N1
 * @value 150 150 8N1
 * @value 200 200 8N1
 * @value 300 300 8N1
 * @value 600 600 8N1
 * @value 1200 1200 8N1
 * @value 1800 1800 8N1
 * @value 2400 2400 8N1
 * @value 4800 4800 8N1
 * @value 9600 9600 8N1
 * @value 19200 19200 8N1
 * @value 38400 38400 8N1
 * @value 57600 57600 8N1
 * @value 115200 115200 8N1
 * @value 230400 230400 8N1
 * @value 460800 460800 8N1
 * @value 500000 500000 8N1
 * @value 921600 921600 8N1
 * @value 1000000 1000000 8N1
 * @value 1500000 1500000 8N1
 * @value 2000000 2000000 8N1
 * @value 3000000 3000000 8N1
 * @group Serial
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SER_TEL1_BAUD, 57600);

/**
 * Baudrate for the TELEM 2 Serial Port
 *
 * Configure the Baudrate for the TELEM 2 Serial Port.
 *
 * Note: certain drivers such as the GPS can determine the Baudrate automatically.
 *
 * @value 0 Auto
 * @value 50 50 8N1
 * @value 75 75 8N1
 * @value 110 110 8N1
 * @value 134 134 8N1
 * @value 150 150 8N1
 * @value 200 200 8N1
 * @value 300 300 8N1
 * @value 600 600 8N1
 * @value 1200 1200 8N1
 * @value 1800 1800 8N1
 * @value 2400 2400 8N1
 * @value 4800 4800 8N1
 * @value 9600 9600 8N1
 * @value 19200 19200 8N1
 * @value 38400 38400 8N1
 * @value 57600 57600 8N1
 * @value 115200 115200 8N1
 * @value 230400 230400 8N1
 * @value 460800 460800 8N1
 * @value 500000 500000 8N1
 * @value 921600 921600 8N1
 * @value 1000000 1000000 8N1
 * @value 1500000 1500000 8N1
 * @value 2000000 2000000 8N1
 * @value 3000000 3000000 8N1
 * @group Serial
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SER_TEL2_BAUD, 921600);


/**
 * Serial Configuration for Lanbao PSK-CM8JL65-CC5
 *
 * Configure on which serial port to run Lanbao PSK-CM8JL65-CC5.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_CM8JL65_CFG, 0);

/**
 * Serial Configuration for LeddarOne Rangefinder
 *
 * Configure on which serial port to run LeddarOne Rangefinder.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_LEDDAR1_CFG, 0);

/**
 * Serial Configuration for Lightware Laser Rangefinder (serial)
 *
 * Configure on which serial port to run Lightware Laser Rangefinder (serial).
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_SF0X_CFG, 0);

/**
 * Serial Configuration for Benewake TFmini Rangefinder
 *
 * Configure on which serial port to run Benewake TFmini Rangefinder.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_TFMINI_CFG, 0);

/**
 * Serial Configuration for uLanding Radar
 *
 * Configure on which serial port to run uLanding Radar.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_ULAND_CFG, 0);

/**
 * Serial Configuration for Secondary GPS
 *
 * Configure on which serial port to run Secondary GPS.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group GPS
 * @reboot_required true
 */
PARAM_DEFINE_INT32(GPS_2_CONFIG, 0);

/**
 * Serial Configuration for Main GPS
 *
 * Configure on which serial port to run Main GPS.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group GPS
 * @reboot_required true
 */
PARAM_DEFINE_INT32(GPS_1_CONFIG, 201);

/**
 * Serial Configuration for ThoneFlow-3901U optical flow sensor
 *
 * Configure on which serial port to run ThoneFlow-3901U optical flow sensor.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(SENS_TFLOW_CFG, 0);

/**
 * Serial Configuration for Roboclaw Driver
 *
 * Configure on which serial port to run Roboclaw Driver.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Roboclaw
 * @reboot_required true
 */
PARAM_DEFINE_INT32(RBCLW_SER_CFG, 0);

/**
 * Serial Configuration for FrSky Telemetry
 *
 * Configure on which serial port to run FrSky Telemetry.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Telemetry
 * @reboot_required true
 */
PARAM_DEFINE_INT32(TEL_FRSKY_CONFIG, 0);

/**
 * Serial Configuration for HoTT Telemetry
 *
 * Configure on which serial port to run HoTT Telemetry.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group Telemetry
 * @reboot_required true
 */
PARAM_DEFINE_INT32(TEL_HOTT_CONFIG, 0);

/**
 * Serial Configuration for MAVLink (instance 0)
 *
 * Configure on which serial port to run MAVLink.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group MAVLink
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_0_CONFIG, 101);

/**
 * Serial Configuration for MAVLink (instance 1)
 *
 * Configure on which serial port to run MAVLink.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group MAVLink
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_1_CONFIG, 0);

/**
 * Serial Configuration for MAVLink (instance 2)
 *
 * Configure on which serial port to run MAVLink.
 *
 * 
 *
 * @value 0 Disabled
 * @value 201 GPS 1
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @group MAVLink
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_2_CONFIG, 0);


