/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PSoC 4 CapSense Pipeline
* Scanning code example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY    (3u)
#define CY_ASSERT_FAILED          (0u)

/* EZI2C interrupt priority must be higher than CapSense interrupt. */
#define EZI2C_INTR_PRIORITY       (2u)

#define NUMBER_OF_SLIDER_SEGMENTS   (6u)
#define NUMBER_OF_BUTTONS			(3u)

/*LED STATE TOGGLE MACROS*/
#define LED_STATE_ON	(0u)
#define LED_STATE_OFF	(1u)


/*******************************************************************************
* Global Definitions
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;

#if CY_CAPSENSE_BIST_EN
/* Variables to hold sensor parasitic capacitances */
uint32_t sensor_cp0 = 0;
uint32_t sensor_cp1 = 0;
uint32_t sensor_cp2 = 0;
uint32_t sense_cap[NUMBER_OF_SLIDER_SEGMENTS] = {0};
cy_en_capsense_bist_status_t status,measure_status[NUMBER_OF_SLIDER_SEGMENTS];
#endif /* CY_CAPSENSE_BIST_EN */

/*Variables to hold LED Port and Pins for Buttons and SLiders*/
GPIO_PRT_Type * LED_PORT_BUTT[NUMBER_OF_BUTTONS] = {P5_5_PORT, P5_7_PORT, P5_2_PORT};
uint8_t LED_PIN_BUTT[NUMBER_OF_BUTTONS] = {P5_5_PIN, P5_7_PIN, P5_2_PIN};

GPIO_PRT_Type * LED_PORT_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
                    {P1_0_PORT,P1_2_PORT,P1_4_PORT,P1_6_PORT,P2_0_PORT,P2_2_PORT};
uint8_t LED_PIN_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
                        {P1_0_PIN,P1_2_PIN,P1_4_PIN,P1_6_PIN,P2_0_PIN,P2_2_PIN};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void initialize_capsense(void);
static void capsense_isr(void);
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
static void led_control(uint8_t WidgetID);

#if CY_CAPSENSE_BIST_EN
static void measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - perform Cp measurement if Built-in Self test (BIST) is enabled
*  - scan touch input continuously
*
* Return:
*  int
*
* Parameters:
*  void
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize CapSense */
    initialize_capsense();

    uint8_t currentWidgetID, previousWidgetID;

    currentWidgetID = 0u;
    previousWidgetID = 0u;
    uint8_t numWgt = cy_capsense_context.ptrCommonConfig->numWd;

    /* Start the first scan of Previous Widget*/
    Cy_CapSense_ScanWidget(previousWidgetID,&cy_capsense_context);

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /*Increase the currentWidgetID by 1 i.e, point to the next Widget*/
            currentWidgetID = (currentWidgetID < (numWgt - 1u))?(currentWidgetID + 1u):0;

            /*Start the first scan of Current Widget*/
            Cy_CapSense_ScanWidget(currentWidgetID,&cy_capsense_context);

            /* Process the Previous Widget */
            Cy_CapSense_ProcessWidget(previousWidgetID,&cy_capsense_context);

            /* Turning ON/OFF based on widget status */
            led_control(previousWidgetID);

            /*Set the previous widget as current widget*/
            previousWidgetID = currentWidgetID;

            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);

#if CY_CAPSENSE_BIST_EN
            /* Measure the self capacitance of sensor electrode using BIST */
            measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */
        }
    }
}


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configures the CapSense
*  interrupt.
*
* Return:
*  void
*
* Parameters:
*  void
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_status status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t capsense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CYRET_SUCCESS == status)
    {
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CYRET_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file
         */
    }
}


/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
* Return:
*  void
*
* Parameters:
*  void
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  EZI2C module to communicate with the CapSense Tuner tool.
*
* Return:
*  void
*
* Parameters:
*  void
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8 *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation. */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

    /* EZI2C initialization failed */
    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}


/*******************************************************************************
* Function Name: led_control
********************************************************************************
* Summary:
*  Toggle the LEDs based on the Active status of CapSense Widgets and their respective Sensors
*
* Return:
*  void
*
* Parameters:
*  WidgetID
*******************************************************************************/
static void led_control(uint8_t WidgetID)
{
    if(0 != Cy_CapSense_IsWidgetActive(WidgetID, &cy_capsense_context))
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {
            for(uint8_t sens_id = 0 ; sens_id < NUMBER_OF_SLIDER_SEGMENTS; sens_id++)
            {
                if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,sens_id,&cy_capsense_context))
                {
                    Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_ON);
                }
                else
                {
                    Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_OFF);
                }
            }
        }
        else{
            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_ON);
        }
    }
    else
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {
            for(uint8_t sens_id = 0 ; sens_id < NUMBER_OF_SLIDER_SEGMENTS; sens_id++)
            {
                Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_OFF);
            }
        }
        else{
            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_OFF);
        }
    }
}

/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from EZI2C block.
*
* Return:
*  void
*
* Parameters:
*  void
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}


#if CY_CAPSENSE_BIST_EN
/*******************************************************************************
* Function Name: measure_sensor_cp
********************************************************************************
* Summary:
*  Measures the self capacitance of the sensor electrode (Cp) in Femto Farad and
*  stores its value in the variable sensor_cp.
*
* Return:
*  void
*
* Parameters:
*  void
*******************************************************************************/
static void measure_sensor_cp()
{
    /* Measure the self capacitance of sensor electrode */
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON0_WDGT_ID,
                CY_CAPSENSE_BUTTON0_SNS0_ID, &sensor_cp0, &cy_capsense_context);
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON1_WDGT_ID,
                CY_CAPSENSE_BUTTON1_SNS0_ID, &sensor_cp1, &cy_capsense_context);
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON2_WDGT_ID,
                CY_CAPSENSE_BUTTON2_SNS0_ID, &sensor_cp2, &cy_capsense_context);
    for(uint8_t sensor_id = CY_CAPSENSE_LINEARSLIDER0_SNS0_ID; sensor_id < CY_CAPSENSE_NUM_SNS_VALUE; sensor_id++){
        measure_status[sensor_id] =
                Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                            sensor_id,&sense_cap[sensor_id], &cy_capsense_context);
    }
}
#endif /* CY_CAPSENSE_BIST_EN */


/* [] END OF FILE */
