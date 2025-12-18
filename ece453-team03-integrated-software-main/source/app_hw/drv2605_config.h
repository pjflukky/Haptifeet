/**
 * @file drv2605_config.h
 * @author Isak Keyes
 * @brief Configuration header for the DRV2605 haptic driver
 * 
 * This file contains the actuator-specific configuration parameters
 * for the DRV2605 haptic driver.
 * 
 * @date 2025-11-05
 */
#ifndef __DRV2605_CONFIG_H__
#define __DRV2605_CONFIG_H__

/**
 * ============[ Background Information]============
 * 
 * Actuator Types:
 * - Linear Resonant Actuator (LRA): Vibration is produced by applying AC signal to
 *      a voice coil which moves a spring-mounted mass.
 *    > Adjustable amplitude, essentially fixed frequency (freq. can technically be controlled
 *      but deviating from resonant frequency greatly reduces performance and efficiency)
 * - Eccentric Rotating Mass (ERM): Vibration is produced by spinning an off-center mass
 *      using a DC motor.
 *    > Frequency and amplitude are coupled
 *    > slower response time due to motor inertia
 * 
 * Other Actuator Types (not being used here):
 * - Linear Magnetic Ram (LMR): A wideband voice coil actuator similar to LRA but
 *      without a spring-mounted mass. This is a newer technology and although very promising,
 *      it is currently prohibitively expensive. (See Titan Haptics "TacHammer" line)
 * - Piezoelectric Actuator: Uses piezoelectric materials to create vibration by mechanical
 *      deformation. These are best for short pulses, and have relatively low displacement.
 * 
 * ============[ Actuator Parameter Information]===========
 * Note: The information below only pertains to closed-loop operation of LRA and ERM actuators.
 * 
 * Actuator Configuration Parameters:
 * - N_ERM_LRA: Selects between ERM (0) and LRA (1) operation
 * - Rated Voltage: Maximum steady-state voltage for the actuator
 * - Overdrive Clamp Voltage: Maximum voltage applied during acceleration/deceleration
 * - Drive Time (LRA Mode): Duration of 1/2 period at the resonant frequency
 * - Drive Time (ERM Mode): Sample rate of back-EMF detection
 * TODO: FB_BRAKE_FACTOR, LOOP_GAIN, etc.?
 * 
 * LRA Parameter Calculations:
 * - Rated Voltage = (255/5.3)(V_rms*sqrt(1-lra_resonant_frequency*(4*t_s + 300e-6)))
 *    > V_rms: maximum steady-state rms voltage specified by manufacturer
 *    > lra_resonant_frequency: Resonant frequency of the LRA in Hz
 *    > t_s: Sample time in seconds (default is 300us)
 * - Overdrive Clamp Voltage = V_peak * (255/5.6)
 *    > V_peak: Maximum voltage applied during acceleration/deceleration, if max voltage
 *              is specified in rms use V_peak=V_rms*sqrt(2)
 * - Drive Time = 5000/lra_resonant_frequency - 5
 *    > lra_resonant_frequency: Resonant frequency of the LRA in Hz
 * 
 * ERM Parameter Calculations:
 * - Rated Voltage = V_dc * (255/5.6)
 *    > V_dc: Maximum steady-state DC voltage specified by manufacturer
 * - Overdrive Clamp Voltage
 *   = V_od * ((DriveTime+IDissTime+BlankingTime)/(DriveTime - 300e-6)) * (255/5.44)
 *    > V_od: Maximum voltage applied during acceleration/deceleration
 *    > IDissTime: = 75e-6 s (default)
 *    > BlankingTime: = 75e-6 s (default)
 *    > DriveTime: = 0x13 = 19 (default)
 */

// Values used to select from predefined actuator configurations
#define CONFIG_VG0832013D 0 // 235Hz 1G LRA: VG0832013D
#define CONFIG_VL91022_160_320H 1 // 160/320Hz 3G LRA: VL91022-160-320H (bidirectional)
#define CONFIG_VLV281813_65H 2 // 65Hz 4.8G LRA: VLV281813-65H
#define CONFIG_VLV101040A 3 // 170Hz 2.75G LRA: VLV101040A (small square)
#define CONFIG_VG2080001_175H 4 // 175Hz 3.2G LRA: VG2080001-175H (large coin)
#define CONFIG_VG2230001H 5 // 70Hz 3.5G LRA: VG2230001H (eq. to PS5 controller haptic)
#define CONFIG_VZ7AL2B1692082 6 // 12kRPM 5.1G ERM: VZ7AL2B1692082

// Select the desired configuration here
#define CONFIG CONFIG_VZ7AL2B1692082

// Logic to define actuator-specific parameters based on selected configuration
#if (CONFIG == CONFIG_VG0832013D)
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x45 // 1.8 V_rms
    #define OD_CLAMP 0x77 // 1.85 V_rms
    #define DRIVE_TIME 0x10 // 235 Hz
#elif (CONFIG == CONFIG_VL91022_160_320H)
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x3E
    #define OD_CLAMP 0x6A
    #define DRIVE_TIME 0x1A
#elif (CONFIG == CONFIG_VLV281813_65H)
    // NOTE: This actuator has a resonant frequency (65Hz) below the recommended minimum
    // for the DRV2605 (125Hz).
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x5B
    #define OD_CLAMP 0x87
    #define DRIVE_TIME 0x1F // 0x47 is actual calculated value but max is 0x1F
#elif (CONFIG == CONFIG_VLV101040A)
    // NOTE: This actuator has a very wide freq. bandwidth (140-300Hz) so it
    // should ideally be driven in the open-loop mode instead of closed-loop.
    // https://www.vybronics.com/linear-lra-vibration-motors/v-lv101040a
    #warning "VLV101040A is best driven in open-loop mode due to wide frequency bandwidth."
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x67
    #define OD_CLAMP 0xA0
    #define DRIVE_TIME 0x18
#elif (CONFIG == CONFIG_VG2080001_175H)
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x4A
    #define OD_CLAMP 0x80
    #define DRIVE_TIME 0x17
#elif (CONFIG == CONFIG_VG2230001H)
    // NOTE: This actuator has a resonant frequency (70Hz) below the recommended minimum
    // for the DRV2605 (125Hz).
    #define N_ERM_LRA 1 // LRA
    #define RATED_VOLTAGE 0x5B
    #define OD_CLAMP 0xC1
    #define DRIVE_TIME 0x1F // 0x42 is actual calculated value but max is 0x1F
#elif (CONFIG == CONFIG_VZ7AL2B1692082)
    #define N_ERM_LRA 0 // ERM
    // #define RATED_VOLTAGE 0x90 // value from setup guide (for 3V)
    // #define RATED_VOLTAGE 0x8D // 3.0V
    #define RATED_VOLTAGE 0x5E // 2.5V
    // #define OD_CLAMP 0xA4 // value from setup guide (for 3.6V)
    // #define OD_CLAMP 0xA2 // 3.2V
    #define OD_CLAMP 0x65// 2.5V
    #define DRIVE_TIME 0x13 // Default for ERM
#else
    #error "Invalid DRV2605 configuration selected in drv2605_config.h"
#endif

// Default definitions for parameters that aren't necessarily actuator-specific
#ifndef FB_BRAKE_FACTOR
    #define FB_BRAKE_FACTOR 3 // = 2^(FB_BRAKE_FACTOR)
#endif
#ifndef LOOP_GAIN
    #define LOOP_GAIN 1 // 0-3 -> low, medium, high, very high
#endif

// verify that all necessary parameters are defined
#ifndef N_ERM_LRA
    #error "N_ERM_LRA not defined"
#endif
#ifndef RATED_VOLTAGE
    #error "RATED_VOLTAGE not defined"
#endif
#ifndef OD_CLAMP
    #error "OD_CLAMP not defined"
#endif
#ifndef DRIVE_TIME
    #error "DRIVE_TIME not defined"
#endif

// Un-define selection definitions to avoid unnecessary namespace pollution
#undef CONFIG_VG0832013D
#undef CONFIG_VL91022_160_320H
#undef CONFIG_VLV281813_65H
#undef CONFIG_VLV101040A
#undef CONFIG_VG2080001_175H
#undef CONFIG_VG2230001H
#undef CONFIG_VZ7AL2B1692082
#undef CONFIG

#endif /* __DRV2605_CONFIG_H__ */