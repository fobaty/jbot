/* Golden-vector recorder for the pinned EFeru FOC controller.
 *
 * Compiles the unmodified generated C for the host and writes one CSV row per
 * 16 kHz tick. The Rust port replays these rows in unit tests so the port can
 * be proven bit-exact instead of "similar enough".
 */
#include "BLDC_controller.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern P rtP_Left;

#define TICKS 120
#define TRQ_MODE_REQ 3U

static const uint8_t hall_sequence[6] = {1U, 3U, 2U, 6U, 4U, 5U};

static void configure(P *p, uint8_t phase_selection) {
    /* Values applied by util.c:232-247 and the pinned jbot limits. */
    p->z_ctrlTypSel = 2U;
    p->b_diagEna = 1U;
    p->b_angleMeasEna = 0U;
    p->z_selPhaCurMeasABC = phase_selection;
    p->b_fieldWeakEna = 0U;
    p->i_max = 8000;
    p->n_max = 6400;
    p->id_fieldWeakMax = 4000;
    p->a_phaAdvMax = 400;
    p->r_fieldWeakHi = 16000;
    p->r_fieldWeakLo = 12000;
    p->n_stdStillDet = 48;
    p->r_errInpTgtThres = 9600;
    p->t_errQual = 1280U;
    p->t_errDequal = 9600U;
    p->Vd_max = 14400;
}

int main(void) {
    RT_MODEL model;
    ExtU inputs;
    ExtY outputs;
    DW dwork;

    memset(&dwork, 0, sizeof(dwork));
    memset(&inputs, 0, sizeof(inputs));
    memset(&outputs, 0, sizeof(outputs));
    configure(&rtP_Left, 1U);

    model.defaultParam = &rtP_Left;
    model.inputs = &inputs;
    model.outputs = &outputs;
    model.dwork = &dwork;
    BLDC_controller_initialize(&model);

    printf("tick,b_motEna,ctrlModReq,inpTgt,hallA,hallB,hallC,phaAB,phaBC,"
           "DC_phaA,DC_phaB,DC_phaC,errCode,n_mot,elecAngle,iq,id\n");
    for (int tick = 0; tick < TICKS; ++tick) {
        const uint8_t hall = hall_sequence[(tick / 8) % 6];
        int16_t target;
        if (tick < 12) {
            target = 0;
        } else if (tick < 36) {
            target = 1000;
        } else if (tick < 60) {
            target = -1000;
        } else if (tick < 84) {
            target = 500;
        } else {
            target = 0;
        }
        /* A synthetic balanced two-phase current, in raw ADC counts. */
        const int16_t phase = (int16_t)((tick * 320) % 4096);
        const int16_t pha_ab = (int16_t)(phase - 2048);
        const int16_t pha_bc = (int16_t)(((phase + 1365) % 4096) - 2048);

        inputs.b_motEna = 1U;
        inputs.z_ctrlModReq = TRQ_MODE_REQ;
        inputs.r_inpTgt = target;
        inputs.b_hallA = (uint8_t)((hall >> 2) & 1U);
        inputs.b_hallB = (uint8_t)((hall >> 1) & 1U);
        inputs.b_hallC = (uint8_t)(hall & 1U);
        inputs.i_phaAB = pha_ab;
        inputs.i_phaBC = pha_bc;
        inputs.i_DCLink = 0;
        inputs.a_mechAngle = 0;

        BLDC_controller_step(&model);

        printf("%d,%u,%u,%d,%u,%u,%u,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d\n",
               tick, (unsigned)(inputs.b_motEna), (unsigned)(inputs.z_ctrlModReq),
               (int)(inputs.r_inpTgt), (unsigned)(inputs.b_hallA),
               (unsigned)(inputs.b_hallB), (unsigned)(inputs.b_hallC),
               (int)(inputs.i_phaAB), (int)(inputs.i_phaBC),
               (int)(outputs.DC_phaA), (int)(outputs.DC_phaB), (int)(outputs.DC_phaC),
               (unsigned)(outputs.z_errCode), (int)(outputs.n_mot),
               (int)(outputs.a_elecAngle), (int)(outputs.iq), (int)(outputs.id));
    }
    return 0;
}
