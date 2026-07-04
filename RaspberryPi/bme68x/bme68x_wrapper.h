#include "bme68x.h"

#ifndef BME68X_WRAPPER_H_
#define BME68X_WRAPPER_H_

/***********************************************************************/
/*                         Macros                                      */
/***********************************************************************/

/* Macro for count of samples to be displayed */
#define SAMPLE_COUNT  UINT8_C(30)

struct bme68x_avg {
    double temperature;
    double humidity;
    double pressure;
    double gas_resistance;
    uint8_t status;
    uint8_t gas_index;
    uint8_t meas_index;
};

extern int bme68x_forced_mode(struct bme68x_dev *bme, struct bme68x_avg * results); 

extern int bme68x_parallel_mode(struct bme68x_dev *bme, struct bme68x_avg *results); 

extern int bme68x_sequential_mode(struct bme68x_dev *bme, struct bme68x_avg * results);

#endif
