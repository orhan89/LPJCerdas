#ifndef lpj_h
#define lpj_h

#define TIDAK_ADA 1
#define ADA 0

enum LPJSignal {
    TickTime_SIG = Q_USER_SIG,
    LowLight_SIG,
    HighLight_SIG,
    reportBefore_SIG,
    reportAfter_SIG,
    PIRIntr_SIG,
    MAX_SIG
};

typedef struct reportTag {
    QEvent super;
    uint8_t presence;
} reportEvt;

typedef struct pirTag {
    QEvent super;
    uint8_t pir;
} pirEvt;

/* typedef struct luxTag { */
/*     QEvent super; */
/*     uint8_t lux; */
/* } luxEvt; */

void LPJ_ctor(void);

extern QActive * const AO_LPJ;

#endif
