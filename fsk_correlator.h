#ifndef _FSK_CORRELATOR_H
#define _FSK_CORRELATOR_H

#include <stdint.h>

#define BIT_MARK     (1)
#define BIT_SPACE    (-1)
#define BIT_IDLE     (0)
#define BIT_DECIDING (2)


typedef struct {
    // NCO state
    int32_t  markPointer, spacePointer;    // Phase accumulator for MARK and SPACE (Q31)
    int32_t  markStep,    spaceStep;       // NCO tuning word for MARK and SPACE (Q31)
    float    fskScalar;                    // = 1.0f / 2^31  (≈4.6566129e-10)

    // correlator accumulators
    float markSineSum,   markCosineSum;
    float spaceSineSum,  spaceCosineSum;
} FskCorrState;

void    fsk_corr_init(FskCorrState *);
void    fsk_corr_step(volatile float, FskCorrState *);
int16_t fsk_corr_detect(FskCorrState *, float);

#endif
