/*
 * chutil.h
 *
 *  Created on: 01.01.2017
 *      Author: joerg
 */



/**
 * @file    chutil.h
 * @author  Austin Glaser <austin.glaser@mgmail.com>
 *
 * Copyright (C) 2016 Austin Glaser
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 *
 * @brief   Shared utility macros and functions
 */

#ifndef SRC_CHUTIL_H_
#define SRC_CHUTIL_H_

/**
 * @defgroup UTIL Utilities
 * @{
 */

/* --- PUBLIC MACROS -------------------------------------------------------- */

/**
 * @brief   Concatenates two symbols s1 and s2 exactly, without expanding either
 *
 * @param[in] s1:       The first symbol to concatenate
 * @param[in] s2:       The second symbol to concatenate
 *
 * @return              A single symbol containing s1 and s2 concatenated without expansion
 */
#define CONCAT_SYMBOLS(s1, s2)     s1##s2

/**
 * @brief  Concatenate the symbols s1 and s2, expanding both of them
 *
 * This is important because simply applying s1##s2 doesn't expand them if they're
 * preprocessor tokens themselves
 *
 * @param[in] s1:       The first symbol to concatenate
 * @param[in] s2:       The second symbol to concatenate
 *
 * @return              A single symbol containing s1 expanded followed by s2 expanded
 */
#define CONCAT_EXPANDED_SYMBOLS(s1, s2)  CONCAT_SYMBOLS(s1, s2)

/** @} defgroup UTIL */


#endif /* SRC_CHUTIL_H_ */
