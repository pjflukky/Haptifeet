/**
 * @file task_core.c
 * @author Isak Keyes, ... TODO
 * @brief Core task which links the LiDAR, potentiometer, and haptics tasks.
 *
 * This task gets LiDAR and potentiometer readings from their respective tasks,
 * then processes the data to determine appropriate haptic feedback, and
 * requests haptic amplitude updates accordingly.
 *
 * @date 2025-12-02
 */
#include "task_core.h"
#include "task_lidar.h" //TODO uncomment and update name once LiDAR task is implemented
#include "task_poten.h"
#include "task_haptics.h"

#define TASK_CORE_STACK_SIZE (configMINIMAL_STACK_SIZE * 20) // TODO how to decide stack size?
#define TASK_CORE_PRIORITY (configMAX_PRIORITIES - 5)        // TODO how to decide priority?

#define FOV_DEGREES 120.0f       // relevant FOV of LiDAR sensor in degrees
#define FOV_CENTER_DEGREES 90.0f // center angle of FOV

#define MIN_HAPTIC_AMPLITUDE 127 // minimum haptic amplitude TODO - verify
#define MAX_HAPTIC_AMPLITUDE 255 // maximum haptic amplitude

#if 0
    #define MIN_ACTIVE_RANGE 700.0f // minimum distance (mm) for haptics to activate
    #define MAX_ACTIVE_RANGE 4000.0f // maximum distance (mm) for haptics to activate
#else
    #define MIN_ACTIVE_RANGE 100.0f // minimum distance (mm) for haptics to activate
    #define MAX_ACTIVE_RANGE 2000.0f // maximum distance (mm) for haptics to activate
#endif
// TODO - increase max active range if logarithmic mapping is used

#define USE_POTEN_INPUT 1  // set to 0 to disable potentiometer input (for testing)
#define POTEN_DEADZONE 0.05f // potentiometer cutoff, below which haptics are disabled


//TODO - non-linear mapping parameters?

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

static void task_core(void *param);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/

// Struct to hold LiDAR points for processing
struct
{
    point_t points[LIDAR_MAX_POINTS];
    int length; // number of valid points
} lidar_scan;   // TODO - move struct to typedef used throughout lidar stuff?

// Latest Potentiometer value
static float poten_value = 0.0f;

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Updates the LiDAR data with a new scan.
 */
static void update_lidar_data(void)
{
    lidar_read(lidar_scan.points);
    // set lidar_scan.length by counting valid points
    int count = 0;
    for (int i = 0; i < LIDAR_MAX_POINTS; i++)
    {
        if (lidar_scan.points[i].valid)
        {
            count++;
        }
        else
        {
            break;
        }
    }
    lidar_scan.length = count;
}

/**
 * @brief Updates the potentiometer value from the potentiometer task.
 */
static void update_poten_value(void)
{
    // Always gets the latest value
    if (xQueueReceive(poten_queue, &poten_value, 0) == pdTRUE)
    {
        // Set to zero if reading is below cutoff
        if (poten_value < POTEN_DEADZONE)
        {
            poten_value = 0.0f;
        }
    }
}

/**
 * @brief Calculates the distance corresponding to each haptic based on the LiDAR data.
 * 
 * TODO - switch from closest point to average or some other metric?
 *
 * @param haptic_dists Array of length NUM_HAPTICS to place calculated distances in
 */
static void calc_haptic_dists(float *const haptic_dists)
{
    float start_angle = FOV_CENTER_DEGREES - (FOV_DEGREES / 2.0f);
    float angle_per_haptic = FOV_DEGREES / NUM_HAPTICS;
    // initialize all distances to out of range values
    for (int i = 0; i < NUM_HAPTICS; i++) {
        haptic_dists[i] = LIDAR_MAX_RANGE + 1.0f;
    }
    // iterate through LiDAR points and assign distances to haptics
    // (currently just uses closest point from each haptic's FOV segment)
    for (int i = 0; i < lidar_scan.length; i++) {
        point_t pt = lidar_scan.points[i];
        // determine which haptic this point corresponds to
        float relative_angle = pt.angle - start_angle;
        if (relative_angle < 0) {
            continue; // point is out of relevant FOV
        } else if (relative_angle > FOV_DEGREES) {
            continue; // all points within FOV have been processed
        }
        int haptic_idx = (int)(relative_angle / angle_per_haptic);
        if (haptic_idx < 0 || haptic_idx >= NUM_HAPTICS) {
            continue; // should not happen, but just in case
        }
        // update haptic distance if this point is closer
        if (pt.dist < haptic_dists[haptic_idx]) {
            haptic_dists[haptic_idx] = pt.dist;
        }
    }
}

/**
 * @brief Maps a distance to a haptic amplitude percentage.
 *
 * @param dist Distance in mm
 * @return float The calculated amplitude percentage (0.0 to 1.0)
 */
static float map_dist_to_amp_pct(float dist)
{
    // return 0 if distance is out of active range
    if (dist < MIN_ACTIVE_RANGE || dist > MAX_ACTIVE_RANGE)
    {
        return 0;
    }
    // linear mapping for now
    float amp_pct = (dist - MIN_ACTIVE_RANGE) / (MAX_ACTIVE_RANGE - MIN_ACTIVE_RANGE);
    return (1.0f - amp_pct);
}

/**
 * @brief Converts calculated distances to haptic amplitudes, incorporating potentiometer value.
 * 
 * @param haptic_dists Array of length NUM_HAPTICS with calculated distances
 * @param haptic_amps Array of length NUM_HAPTICS to place calculated amplitudes in
 * 
 */
static void map_haptic_amps(const float *const haptic_dists, uint8_t *const haptic_amps)
{
    uint8_t amp_range = MAX_HAPTIC_AMPLITUDE - MIN_HAPTIC_AMPLITUDE;

    for (int i = 0; i < NUM_HAPTICS; i++)
    {
        float amp_pct = map_dist_to_amp_pct(haptic_dists[i]);
        // scale by potentiometer value
#if USE_POTEN_INPUT
        amp_pct *= poten_value;
#endif
        // convert to amplitude value
        uint8_t amplitude = (uint8_t)(MIN_HAPTIC_AMPLITUDE + (amp_range * amp_pct));
        haptic_amps[i] = amplitude;
    }
}

/**
 * @brief Sends haptic amplitude update requests to the haptics task.
 * 
 * @param haptic_amps Array of length NUM_HAPTICS with calculated amplitudes
 */
static void send_haptic_updates(const uint8_t *const haptic_amps)
{
    for (int i = 0; i < NUM_HAPTICS; i++)
    {
        req_haptic_update(i, haptic_amps[i]);
    }
}

/**
 * @brief
 * Core task which links the LiDAR, potentiometer, and haptics tasks.
 *
 * This task gets LiDAR and potentiometer readings from their respective tasks,
 * then processes the data to determine appropriate haptic feedback, and
 * requests haptic amplitude updates accordingly.
 */
static void task_core(void *param)
{
    float haptic_dists[NUM_HAPTICS] = {0};

    uint8_t haptic_amps[NUM_HAPTICS] = {0};

    // set initial haptic amplitudes to 0
    send_haptic_updates(haptic_amps);

    while (1)
    {
        update_lidar_data();

        update_poten_value();

        calc_haptic_dists(haptic_dists);

        map_haptic_amps(haptic_dists, haptic_amps);

        send_haptic_updates(haptic_amps);

        // TODO - Delay or wait for notification/event before next iteration?
    }
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Initializes the core task.
 */
void task_core_init(void)
{
    /* Create the task that will handle core processing */
    xTaskCreate(
        task_core,
        "Core",
        TASK_CORE_STACK_SIZE,
        NULL,
        TASK_CORE_PRIORITY,
        NULL);
}
