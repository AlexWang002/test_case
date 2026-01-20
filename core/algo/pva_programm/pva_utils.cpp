/*******************************************************************************
 * \addtogroup pva_utils
 * \{
 * \file pva_utils.cpp
 * \brief
 * \version 0.3
 * \date 2025-11-28
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "pva_utils.h"
#include <chrono>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
/**
 * \brief Get a PVA Stream object instance
 *
 * \return A Stream object instance
 */
Stream& getAlgoStream() {
    static Stream algo_stream = Stream::Create(PVA0, VPU1);
    return algo_stream;
}