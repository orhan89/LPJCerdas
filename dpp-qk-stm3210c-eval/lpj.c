#include "qp_port.h"
#include "lpj.h"
#include "bsp.h"
#include <stdio.h>

Q_DEFINE_THIS_FILE

typedef struct LPJTag {
/* protected: */
    QActive super;

/* private: */
    QTimeEvt timeEvent;
    uint8_t pir;
    uint8_t before;
    uint8_t after;
    uint16_t dimm;
} LPJ;

/* protected: */
static QState LPJ_initial(LPJ * const me, QEvent const * const e);
static QState LPJ_Off(LPJ * const me, QEvent const * const e);
static QState LPJ_On(LPJ * const me, QEvent const * const e);
static QState LPJ_Dimm(LPJ * const me, QEvent const * const e);
static QState LPJ_Bright(LPJ * const me, QEvent const * const e);
static QState LPJ_NodeBefore(LPJ * const me, QEvent const * const e);
static QState LPJ_OnTheNode(LPJ * const me, QEvent const * const e);

static LPJ l_LPJ;

QActive * const AO_LPJ = (QActive *)&l_LPJ;

static QState LPJ_initial(LPJ * const me, QEvent const * const e) {
    QTimeEvt_postEvery(
        &me->timeEvent,
        (QActive *)me,
        BSP_TICKS_PER_SEC/2);

    me->dimm = 50;
    return Q_TRAN(&LPJ_Off);
}
/* @(/1/0/1/1) .............................................................*/
static QState LPJ_Off(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        /* @(/1/0/1/1) */
        case Q_ENTRY_SIG: {
	    //printf("LPJ_Off");
            turnOff();
	    //kirimReport(0x03);
            status = Q_HANDLED();
            break;
        }
        /* @(/1/0/1/1/0) */
        case LowLight_SIG: {
            status = Q_TRAN(&LPJ_Dimm);
            break;
        }

        case TickTime_SIG: {
//            periksaLux();
	    status = Q_HANDLED();
	    break;
	}

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}
/* @(/1/0/1/2) .............................................................*/
static QState LPJ_On(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        /* @(/1/0/1/2) */
        case Q_ENTRY_SIG: {
	    //printf("LPJ_On\r\n");
            turnOn();
	    //kirimReport(0x02);
	    me->dimm = 0;
            status = Q_HANDLED();
            break;
        }
        /* @(/1/0/1/2/0) */
        case HighLight_SIG: {
            status = Q_TRAN(&LPJ_Off);
            break;
        }
        /* @(/1/0/1/2/2/1/0) */
        case reportAfter_SIG: {
	    me->after =((reportEvt const *)e)->presence;
	    status = Q_HANDLED();
            break;
        }
        case reportBefore_SIG: {
	    me->before =((reportEvt const *)e)->presence;
	    status = Q_HANDLED();
	    break;
	}
        case TickTime_SIG: {
//            periksaLux();
	    status = Q_HANDLED();
	    break;
	}
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}
/* @(/1/0/1/2/1) ...........................................................*/
static QState LPJ_Dimm(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        /* @(/1/0/1/2/1) */
        case Q_ENTRY_SIG: {
            /* Dimm(me->dimm); */
	    //printf("LPJ_Dimm");
	    periksaPIR();
            status = Q_HANDLED();
            break;
        }
        /* @(/1/0/1/2/1/0) */
        case reportBefore_SIG: {
	    me->before =((reportEvt const *)e)->presence;
	    if(me->before == ADA) status = Q_TRAN(&LPJ_NodeBefore);
	    else status = Q_HANDLED();
            break;
        }
        /* @(/1/0/1/2/1/1) */
        case PIRIntr_SIG: {
	    me->pir = ((pirEvt const *)e)->pir;
	    //kirimReport((me->pir)+4);
	    if(me->pir == ADA) status = Q_TRAN(&LPJ_OnTheNode);
	    else status = Q_HANDLED();
            break;
        }
        case TickTime_SIG: {
	    if(me->dimm < 3500) {
		me->dimm +=20;
		Dimm(me->dimm);
	    }
//	    periksaLux();
	    status = Q_HANDLED();
	}
        default: {
            status = Q_SUPER(&LPJ_On);
            break;
        }
    }
    return status;
}
/* @(/1/0/1/2/2) ...........................................................*/
static QState LPJ_Bright(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        /* @(/1/0/1/2/2) */
        case Q_ENTRY_SIG: {
            Bright();
	    me->dimm = 0;
            status = Q_HANDLED();
            break;
        }
        case PIRIntr_SIG: {
	    me->pir =((pirEvt const *)e)->pir;
	    //kirimReport((me->pir)+4);
	    status = Q_HANDLED();
	    break;
	}
        default: {
            status = Q_SUPER(&LPJ_On);
            break;
        }
    }
    return status;
}
/* @(/1/0/1/2/2/0) .........................................................*/
static QState LPJ_NodeBefore(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        /* @(/1/0/1/2/2/0/0) */
        case Q_ENTRY_SIG: {
	    //printf("Node Before\r\n");
            status = Q_HANDLED();
            break;
        }
        case TickTime_SIG: {
//	    periksaLux();
	    if((me->before == TIDAK_ADA) && (me->pir == ADA)) status = Q_TRAN(&LPJ_OnTheNode);
	    else status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&LPJ_Bright);
            break;
        }
    }
    return status;
}
/* @(/1/0/1/2/2/1) .........................................................*/
static QState LPJ_OnTheNode(LPJ * const me, QEvent const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
	    //printf("OnTheNode\r\n");
            status = Q_HANDLED();
            break;
        }
        /* @(/1/0/1/2/2/1/1) */
       case reportBefore_SIG: {
	    me->before =((reportEvt const *)e)->presence;
	    if(me->before == ADA) status = Q_TRAN(&LPJ_NodeBefore);
	    else status = Q_HANDLED();
            break;
        }
        case reportAfter_SIG: {
	    me->after =((reportEvt const *)e)->presence;
            if(me->pir == TIDAK_ADA && me->after == ADA) status = Q_TRAN(&LPJ_Dimm);
	    else status = Q_HANDLED();
	    break;
	}
        default: {
            status = Q_SUPER(&LPJ_Bright);
            break;
        }
    }
    return status;
}

void LPJ_ctor(void) {
    LPJ *me = &l_LPJ;
    QActive_ctor(&me->super, (QStateHandler)&LPJ_initial);
    QTimeEvt_ctor(&me->timeEvent, TickTime_SIG);
}
