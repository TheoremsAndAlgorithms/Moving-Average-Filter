#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

/* ESP32 pin D34 is used as the ADC input pin */

#define ADC_CHANNEL  ADC_CHANNEL_6
#define ADC_UNIT     ADC_UNIT_1
#define ADC_ATTEN    ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

#define SUP_VOL      3.3f       /* V_S   (Volt)   */
#define RES_UP       4640.0f    /* R_U   (Ohm)    */
#define BETA         3977.0f    /* β     (Kelvin) */
#define RES_25       4700.0f    /* R_T_0 (Ohm)    */
#define TMP_25       298.15f    /* T_0   (Kelvin) */

#define MILLIVOLTS_TO_VOLTS(mv) ((mv)  / 1000.0f)
#define KEL_TO_DEG_C(kel)       ((kel) - 273.15f) 

#define WIN_LEN 25

static const char TAG[] = "main";

typedef struct
{
    float win[WIN_LEN];

    uint8_t idx;
    uint8_t cnt;

    float sumOfSqrs;
    float sumOfCubes;

    float avg;
    float std;
    float skwns;

} maf_t;

static adc_oneshot_unit_handle_t _hUnit = NULL;
static adc_cali_handle_t         _hCali = NULL;

void ADC_Init(void)
{
    adc_cali_line_fitting_config_t caliConfig =
    {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&caliConfig, &_hCali));

    adc_oneshot_unit_init_cfg_t unitConfig =
    {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &_hUnit));

    adc_oneshot_chan_cfg_t chConfig =
    {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_hUnit, ADC_CHANNEL, &chConfig));
}

float ADC_GetVoltage_V(void)
{
    int adcVal_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(_hUnit, ADC_CHANNEL, &adcVal_raw));

    int vol_mV = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_hCali, adcVal_raw, &vol_mV));

    return MILLIVOLTS_TO_VOLTS(vol_mV);
}

float NTC_GetTemperature_degC(void) // From NTCLE100E3472JB0
{
    float vol_V   = ADC_GetVoltage_V();
    float res_ohm = RES_UP * vol_V / (SUP_VOL - vol_V);
    float tmp_K   = BETA * TMP_25 / (TMP_25 * logf(res_ohm / RES_25) + BETA);

    return KEL_TO_DEG_C(tmp_K);
}

esp_err_t MAF_Init(maf_t *pMaf)
{
    if(pMaf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(pMaf->win, 0, sizeof(pMaf->win));

    pMaf->idx = 0;
    pMaf->cnt = 0;

    pMaf->sumOfSqrs  = 0;
    pMaf->sumOfCubes = 0;
   
    pMaf->avg   = 0;
    pMaf->std   = 0;
    pMaf->skwns = 0;

    return ESP_OK;
}

esp_err_t MAF_Update(maf_t *pMaf, float smp)
{
    if(pMaf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bool isFull = pMaf->cnt == WIN_LEN;

    pMaf->cnt++;
    pMaf->cnt = fminf(pMaf->cnt, WIN_LEN);

    float smpOld  = isFull ? pMaf->win[pMaf->idx] : 0;
    float avgPrev = pMaf->avg;

    float deltaAvg = isFull ? smp - smpOld : smp - avgPrev;
    deltaAvg      /= pMaf->cnt;
    pMaf->avg     += deltaAvg;

    float deltaSumOfSqrs_1 =  (smp    - pMaf->avg) * (smp    - avgPrev);
    float deltaSumOfSqrs_2 = -(smpOld - pMaf->avg) * (smpOld - avgPrev);

    pMaf->sumOfSqrs += deltaSumOfSqrs_1 + isFull * deltaSumOfSqrs_2;
    pMaf->std        = (pMaf->cnt > 1) ? sqrtf(pMaf->sumOfSqrs / (pMaf->cnt - 1)) : 0;

    float temp        = deltaAvg - avgPrev;
    pMaf->sumOfCubes += deltaSumOfSqrs_1 * (smp + temp) + isFull * deltaSumOfSqrs_2 * (smpOld + temp) - 3 * pMaf->sumOfSqrs * deltaAvg;
    pMaf->skwns       = (pMaf->cnt >= 3 && pMaf->std > 0) ? pMaf->cnt / ((pMaf->cnt - 2) * pMaf->sumOfSqrs * pMaf->std) * pMaf->sumOfCubes : 0;

    pMaf->win[pMaf->idx] = smp;

    pMaf->idx++;
    pMaf->idx %= WIN_LEN;

    return ESP_OK;
}

int app_main()
{
    ADC_Init();

    maf_t maf;

    ESP_ERROR_CHECK(MAF_Init(&maf));

    uint32_t cnt = 0;

    struct
    {
        float buff[WIN_LEN];

        uint8_t idx;
        uint8_t cnt;

        float avg;
        float std;
        float skwns;

    } test = {0};

    while(true)
    {
        cnt++;

        float tmp_degC = NTC_GetTemperature_degC();

        ESP_ERROR_CHECK(MAF_Update(&maf, tmp_degC));

        ESP_LOGI(TAG, "%lu. %s, smp = %.2f, avg = %.2f, std = %.2f, skwns = %.2f", cnt, maf.cnt == WIN_LEN ? "Full" : "Not full", tmp_degC, maf.avg, maf.std, maf.skwns);

        test.buff[test.idx] = tmp_degC;

        test.idx++;
        test.idx %= WIN_LEN;

        if(test.cnt < WIN_LEN)
        {
            test.cnt++;
        }

        test.avg   = 0.0f;
        test.std   = 0.0f;
        test.skwns = 0.0f;

        for(uint8_t i = 0; i < test.cnt; i++)
        {
            test.avg += test.buff[i];
        }
        test.avg /= test.cnt;

        if(test.cnt > 1)
        {
            for(uint8_t i = 0; i < test.cnt; i++)
            {
                test.std += powf(test.buff[i] - test.avg, 2);
            }
            test.std = sqrtf(test.std / (test.cnt - 1));
        }

        if(test.cnt > 2)
        {
            for(uint8_t i = 0; i < test.cnt; i++)
            {
                test.skwns += powf(test.buff[i] - test.avg, 3);
            }
            test.skwns *= test.cnt / ((test.cnt - 2) * (test.cnt - 1) * powf(test.std, 3));
        }

        ESP_LOGI(TAG, "%lu. %s, smp = %.2f, avg = %.2f, std = %.2f, skwns = %.2f\n", cnt, test.cnt == WIN_LEN ? "Full" : "Not full", tmp_degC, test.avg, test.std, test.skwns);

        vTaskDelay(pdMS_TO_TICKS(10));

        if(cnt >= 50)
        {
            break;
        }
    }

    return 0;
}