/**
 *    @file    error_types.h
 *
 *    @brief    header file for error_types
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:38:30 BST 2026
 */

#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H

/** Includes **/

/** Defines **/

/** Typedefs **/

typedef enum __attribute__((short)) {
    STATUS_OK = 0,
    STATUS_ERR_INVALID_ARG,
    STATUS_ERR_NO_MEM,
    STATUS_ERR_TIMEOUT,
    STATUS_NOT_IMPLEMENTED,
} status_t;

/** Function Declarations **/

/** END **/
#endif /** ERROR_TYPES_H **/
