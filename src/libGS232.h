/**
 * @libGS232.h
 *
 * @brief Library for Yaesu Antenna Rotator GS-232 A and B protocol server
 * @details Library for Yaesu Antenna Rotator GS-232 A and B protocol server
 *
 * @author Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * @version 0.1
 * @date 2023
 * @copyright MIT License
 * @see https://github.com/hiperiondev/libGS232
 */

#ifndef LIB_GS232_H_
#define LIB_GS232_H_

#include <stdint.h>
#include <stdbool.h>

#define DEBUG               /*!< debug mode */
#define MEMORY_POINTS  3800 /*!< total memory points */

/**
 * @enum GS232_ERROR
 * @brief Errors
 *
 */
enum GS232_ERROR {
    GS232_FAIL          = 250, /*!< generic error */
    GS232_TOOMANYVALUES = 251, /*!< too many values */
    GS232_OUTOFRANGE    = 252, /*!< value out of range */
    //----------------------//
    GS232_OK            = 255, /*!< ok */
};

/**
 * @enum GS232_COMMAND
 * @brief Parsed command
 *
 */
enum GS232_COMMAND {
    GS232_CLOCKWISE_ROTATION,                             /*!< R */
    GS232_UP_DIRECTION_ROTATION,                          /*!< U */
    GS232_COUNTER_CLOCKWISE_ROTATION,                     /*!< L */
    GS232_DOWN_DIRECTION_ROTATION,                        /*!< D */
    GS232_CW_CCW_ROTATION_STOP,                           /*!< A */
    GS232_UP_DOWN_DIRECTION_ROTATION_STOP,                /*!< E */
    GS232_RETURN_CURRENT_AZIMUTH,                         /*!< C */
    GS232_RETURN_AZIMUTH_AND_ELEVATION,                   /*!< C2 */
    GS232_TURN_DEGREES_AZIMUTH,                           /*!< Maaa */
    GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH,               /*!< Mttt aaa aaa aaa ... */
    GS232_TURN_DEGREES_AZIMUTH_AND_ELEVATION,             /*!< Waaa eee */
    GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH_AND_ELEVATION, /*!< Wttt aaa eee aaa eee ... */
    GS232_TOTAL_NUMBER_OF_SETTING_ANGLES,                 /*!< N */
    GS232_START_COMMAND_IN_TIME_INTERVAL,                 /*!< T */
    GS232_ROTATION_SPEED_LOW,                             /*!< X1 */
    GS232_ROTATION_SPEED_MIDDLE1,                         /*!< X2 */
    GS232_ROTATION_SPEED_MIDDLE2,                         /*!< X3 */
    GS232_ROTATION_SPEED_HIGH,                            /*!< X4 */
    GS232_OFFSET_CALIBRATION_AZIMUTH,                     /*!< O */
    GS232_OFFSET_CALIBRATION_ELEVATION,                   /*!< O2 */
    GS232_FULL_SCALE_CALIBRATION_AZIMUTH,                 /*!< F */
    GS232_FULL_SCALE_CALIBRATION_ELEVATION,               /*!< F2 */
    GS232_RETURN_CURRENT_ELEVATION,                       /*!< B */
    GS232_ALL_STOP,                                       /*!< S */
    GS232_LIST_OF_COMMANDS1,                              /*!< H */
    GS232_LIST_OF_COMMANDS2,                              /*!< H2 */
    GS232_LIST_OF_COMMANDS3,                              /*!< H3 */
    GS232_AZIMUTH_TO_360,                                 /*!< P36 */
    GS232_AZIMUTH_TO_450,                                 /*!< P45 */
    GS232_TOGGLE_AZIMUTH_NORD_SOUTH,                      /*!< Z */
    GS232_UNKNOWN_COMMAND                                 /*!< unknown command */
};

// functions prototype for hardware
/**
 * @fn uint8_t (*rotator_set_azimuth)(uint16_t azimuth)
 * @brief Set azimuth position
 *
 * @param azimuth Azimuth
 * @return OK=0, 1=ERROR
 */
typedef  uint8_t (*rotator_set_azimuth)(uint16_t azimuth);

/**
 * @fn uint16_t (*rotator_get_azimuth)(void)
 * @brief Get azimuth position
 *
 * @return Azimuth
 */
typedef uint16_t (*rotator_get_azimuth)(void);

/**
 * @fn uint8_t (*rotator_set_elevation)(uint16_t elevation)
 * @brief Set elevation position
 *
 * @param Elevation
 * @return OK=0, 1=ERROR
 */
typedef uint8_t (*rotator_set_elevation)(uint16_t elevation);

/**
 * @fn uint16_t (*rotator_get_elevation)(void)
 * @brief Get elevation position
 *
 * @return Elevation
 */
typedef uint16_t (*rotator_get_elevation)(void);

/**
 * @typedef gs232_t
 * @brief Context GS-232 data
 *
 */
typedef struct {
        bool b_protocol;              /*!< is GS-232B */
        bool is_450_degrees;          /*!< is 450 degrees mode */
        bool azimuth_nord_south;      /*!< center [0:north, 1:south] */
     uint8_t rotation_speed;          /*!< from command X */
    uint16_t azimuth;                 /*!< azimuth */
    uint16_t elevation;               /*!< elevation */
    uint16_t memory[MEMORY_POINTS];   /*!< memory */
    uint16_t memory_qty;              /*!< memory used */
    uint16_t current_point;           /*!< currently selected memorized point */
    struct {
          rotator_set_azimuth set_azimuth;
          rotator_get_azimuth get_azimuth;
        rotator_set_elevation set_elevation;
        rotator_get_elevation get_elevation;
    } fn;
} gs232_t;

/**
 * @fn uint8_t gs232_init(gs232_t **ctx)
 * @brief Initialize context
 *
 * @param context Context
 * @return GS232
 */
uint8_t gs232_init(gs232_t **ctx);

/**
 * @fn uint8_t gs232_deinit(gs232_t **gs232)
 * @brief Destroy context
 *
 * @param gs232 Context
 * @return GS232_ERROR
 */
uint8_t gs232_deinit(gs232_t **ctx);

/**
 * @fn uint8_t gs232_parse_command(gs232_t **ctx, char *buffer, uint32_t buffer_len)
 * @brief Parse received command buffer
 *
 * @param context Context
 * @param buffer Received command buffer
 * @param buffer_len Received command buffer length
 * @return Command or GS232_ERROR
 */
uint8_t gs232_parse_command(gs232_t **ctx, char *buffer, uint32_t buffer_len);

/**
 * @fn uint8_t gs232_return_string(gs232_t *ctx, uint8_t command, char *ret_str)
 * @brief Create return string for parsed command buffer
 *
 * @param context Context
 * @param command Parsed command
 * @param ret_str Return string
 * @return GS232_ERROR
 */
uint8_t gs232_return_string(gs232_t *ctx, uint8_t command, char **ret_str);



#endif /* LIB_GS232_H_ */
