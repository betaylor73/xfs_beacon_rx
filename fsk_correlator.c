/******************************************************************************
 * Purpose:
 *   High-performance C equivalent of the original fsk_corr_isr_run.asm
 *   correlation routine used in the SunSpec Rapid Shutdown proof-of-concept.
 *
 * Overview:
 *   This routine is called once per ADC sample (e.g., 300 kHz rate). It forms
 *   the “inner loop” of the non-coherent FSK demodulator.
 *
 *   Each call:
 *     1. Advances two 32-bit phase accumulators (MARK & SPACE) in Q31 format.
 *     2. Converts those integer phase values to per-unit floats [0.0,1.0),
 *        where 1.0 corresponds to 2π radians.
 *     3. Uses fast FPU trig intrinsics (__sinpuf32 / __cospuf32) to generate
 *        in-phase and quadrature reference waves.
 *     4. Multiplies the incoming ADC sample by each reference and accumulates
 *        the results into four running correlation sums:
 *           markSineSum, markCosineSum, spaceSineSum, spaceCosineSum
 *
 *   These four sums are later consumed (and reset) by the bit-rate routine to
 *   decide whether the recent window corresponded to a MARK or SPACE tone.
 *
 * Key concepts:
 *   - Fixed-point NCO (Q31 accumulator, wraps every 2^31 counts)
 *   - Per-unit trig functions: input element of [0.0, 1.0) => angle = 2(pi)·x radians
 *   - Non-coherent correlation: energy = sin^2 + cos^2 projections
 *
 *****************************************************************************/

#include <stdint.h>
#include <math.h>   // not for sin/cos here; we’ll use intrinsics

#include "fsk_correlator.h"

// TI FPU “per-unit” trig intrinsics:
//   __sinpuf32(x) : x element of [0.0,1.0) -> sin(2(pi)x)
//   __cospuf32(x) : x element of [0.0,1.0) -> cos(2(pi)x)
// Each executes as a single FPU instruction (SINPUF32/COSPUF32).
// TI compiler: enable TMU in project: --tmu_support=tmu0
extern float __sinpuf32(float x);  // per-unit angle: 1.0 == 2(pi)
extern float __cospuf32(float x);

static const float mark_freq  = 131250.0f;
static const float space_freq = 143750.0f;
static const float isr_freq   = 300000.0f;
static const float bit_freq   = 586;

static uint16_t mark_counter, space_counter, zero_counter, bit_sample_counter;

void fsk_corr_init(FskCorrState* s)
{
    s->markPointer    = 0;
    s->spacePointer   = 0;
    s->markStep       = (int32_t)((mark_freq/ (float)isr_freq)*2147483648.0);
    s->spaceStep      = (int32_t)((space_freq/ (float)isr_freq)*2147483648.0);
    s->fskScalar      = 4.6566129e-10; //1/2^31
    s->markSineSum    = 0.0;
    s->markCosineSum  = 0.0;
    s->spaceSineSum   = 0.0;
    s->spaceCosineSum = 0.0;

    mark_counter = 0;
    space_counter = 0;
    zero_counter = 0;
    bit_sample_counter = 0;
    
}

#pragma CODE_SECTION(fsk_corr_step, ".TI.ramfunc")
/*static inline*/ void fsk_corr_step(volatile float adc_value, FskCorrState* s)
{
    //----------------------------------------------------------------------
    // 1) Advance MARK and SPACE NCO phase accumulators
    //----------------------------------------------------------------------
    //
    // Each pointer is a 32-bit fixed-point counter representing the phase of
    // its reference tone. On every sample, we add the precomputed tuning word.
    //
    // The mask 0x7FFFFFFF enforces modulo-2^31 arithmetic (Q31 domain).  This
    // mirrors the assembly instruction “AND AH, #0x7FFF” which clears the MSB
    // of the 32-bit register pair, thereby wrapping the accumulator cleanly.
    //
    //----------------------------------------------------------------------
    uint32_t mp = (uint32_t)s->markPointer;
    uint32_t sp = (uint32_t)s->spacePointer;

    mp = (mp + (uint32_t)s->markStep)  & 0x7FFFFFFFu;
    sp = (sp + (uint32_t)s->spaceStep) & 0x7FFFFFFFu;

    s->markPointer  = (int32_t)mp;
    s->spacePointer = (int32_t)sp;

    //----------------------------------------------------------------------
    // 2) Convert Q31 phase → per-unit float for trig intrinsics
    //----------------------------------------------------------------------
    //
    //  markPointer element of [0, 2^31)    →  pu_m element of [0.0, 1.0)
    //  spacePointer element of [0, 2^31)   →  pu_s element of [0.0, 1.0)
    //
    // The per-unit domain is defined such that:
    //     0.0 -> 0 radians
    //     0.25 -> pi/2
    //     0.5 -> pi
    //     0.75 -> 3(pi)/2
    //     1.0 -> wraps back to 0
    //----------------------------------------------------------------------

    // Fixed-point phase -> per-unit angle (apply wrap here)
    float pu_m = ((float)s->markPointer)  * s->fskScalar;
    float pu_s = ((float)s->spacePointer) * s->fskScalar;

    //----------------------------------------------------------------------
    // 3) Compute MARK sin/cos references and update correlation sums
    //----------------------------------------------------------------------
    //
    // These represent the in-phase (cos) and quadrature (sin) projections of
    // the ADC signal onto the MARK reference frequency.
    //
    // The FPU intrinsics translate directly to the hardware SINPUF32/COSPUF32
    // opcodes, each executing in one pipeline slot with minimal latency.
    //----------------------------------------------------------------------
    float sm = __sinpuf32(pu_m);    // sin(2pi·phase_mark)
    float cm = __cospuf32(pu_m);    // cos(2pi·phase_mark)

    s->markSineSum   += adc_value * sm;
    s->markCosineSum += adc_value * cm;

    //----------------------------------------------------------------------
    // 4) Compute SPACE sin/cos references and update correlation sums
    //----------------------------------------------------------------------
    //
    // Same procedure for the second tone. These four accumulators are later
    // used to compute:
    //     E_mark  = markSineSum^2   + markCosineSum^2
    //     E_space = spaceSineSum^2  + spaceCosineSum^2
    // which are compared to a threshold to classify the current bit.
    //----------------------------------------------------------------------
    float ss = __sinpuf32(pu_s);    // sin(2π·phase_space)
    float cs = __cospuf32(pu_s);    // cos(2π·phase_space)

    s->spaceSineSum   += adc_value * ss;
    s->spaceCosineSum += adc_value * cs;
}

int16_t fsk_corr_detect(FskCorrState* s, float detection_threshold)
{
    float temp = (isr_freq) / (bit_freq * 2.0f);
    temp = temp * temp;

    float markSumSquared = (s->markCosineSum * s->markCosineSum + s->markSineSum * s->markSineSum) / temp;
    float spaceSumSquared = (s->spaceCosineSum * s->spaceCosineSum + s->spaceSineSum * s->spaceSineSum) / temp;

    int16_t bit_detected = BIT_IDLE;

    if ((markSumSquared > detection_threshold) || (spaceSumSquared > detection_threshold)) {
        if (markSumSquared > detection_threshold) {
            mark_counter++;
        }
        if (spaceSumSquared > detection_threshold) {
            space_counter++;
        }
        bit_sample_counter++;
    } else {
        zero_counter++;
        if (bit_sample_counter > 0) {
            bit_sample_counter++;
        }
    }

    if (bit_sample_counter > 2) {
        if ((mark_counter > space_counter) && (mark_counter > 1)) {
            bit_detected = BIT_MARK;
        } else if ((space_counter > mark_counter) && (space_counter > 1)) {
            bit_detected = BIT_SPACE;
        } else {
            bit_detected = BIT_IDLE;
        }

        // Reset counters for next detection
        mark_counter = 0;
        space_counter = 0;
        zero_counter = 0;
        bit_sample_counter = 0;
    } else if (bit_sample_counter > 0) {
        bit_detected = BIT_DECIDING;      
    }

    // Reset sums for next detection
    s->markSineSum    = 0.0;
    s->markCosineSum  = 0.0;
    s->spaceSineSum   = 0.0;
    s->spaceCosineSum = 0.0;

    return bit_detected;
}
