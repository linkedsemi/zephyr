#ifndef LS_ADC_H_
#define LS_ADC_H_

/** 
 *  @defgroup The conversion mode of ADC
*/
#define regular_mode  0x00000000U
#define inject_mode   0x00000001U
#define loop_mode     0x00000002U

/** 
 *  @defgroup ADC_Data_align ADC data alignment
*/
#define ADC_DATAALIGN_RIGHT 0x00000000U
#define ADC_DATAALIGN_LEFT ((uint32_t)ADC_DATA_ALIGN_MASK)

#define continuous 1
#define discontinuous 0

/** 
 * @defgroup ADC_External_trigger_edge_Regular ADC external trigger enable 
*/
#define ADC_PIS_TRIG                0x00000000U
#define ADC_SOFTWARE_TRIGT          0x111

#define  ADC_VREF_VCC       0       /*!< system power*/
#define  ADC_VREF_EXPOWER   1       /*!< External power */
#define  ADC_VREF_INSIDE    2       /*!< inside power */

/** 
  * @brief  driving ADC type enumeration definition 
*/
#define EINBUF_DRIVE_ADC                    0            /*!< En Inbuf mode */
#define INRES_ONETHIRD_EINBUF_DRIVE_ADC     1            /*!< 1/3 partial voltage input signal */
#define BINBUF_DIRECT_DRIVE_ADC             2            /*!< By Pass Inbuf */

/** 
 *  @defgroup ADC_Clock_Division ADC Clock Division
*/               
#define ADC_CH_CLOCK_DIV1          0x00000000U
#define ADC_CH_CLOCK_DIV2          0x00000001U
#define ADC_CH_CLOCK_DIV4          0x00000002U
#define ADC_CH_CLOCK_DIV8          0x00000003U

#endif /* LS_ADC_H_ */

