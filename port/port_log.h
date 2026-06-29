/**
 *    @file    port_log.h
 *
 *    @brief    header file for port_log
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:43:43 BST 2026
 */

#ifndef PORT_LOG_H
#define PORT_LOG_H

/** Includes **/

/** Defines **/

/** Typedefs **/

/** Function Declarations **/

void __printf(const char *const msg, ...);

void log_fatal(const char *const msg, ...);

void log_error(const char *const msg, ...);

void log_warning(const char *const msg, ...);

void log_info(const char *const msg, ...);

void log_verbose(const char *const msg, ...);

/** END **/
#endif /** PORT_LOG_H **/
