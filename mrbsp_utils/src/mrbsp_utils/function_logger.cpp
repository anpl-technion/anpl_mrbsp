/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (MRBSP),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 * GPLv3
 * -------------------------------------------------------------------------- */

/**
  * @file: function_logger.cpp
  * @brief: defines classes and macros for creating one-line log commands in functions.
  * @author: Nikita Dizhur
  *
  */

#include "mrbsp_utils/function_logger.h"
#include "mrbsp_utils/mrbsp_utils.h"

using namespace MRBSP::Utils;


FunctionLogger::FunctionLogger(LogType type, int log_level, const std::string &func_name, LogTag tag):
        m_type(type), m_log_level(log_level), m_func_name(func_name), m_tag(tag) {
    logMessage(m_type, m_log_level, "start " + m_func_name, m_tag);
}

FunctionLogger::~FunctionLogger() {
    //from: http://www.cplusplus.com/reference/exception/uncaught_exception/
    if(std::uncaught_exception()) {
        //from: http://stackoverflow.com/questions/315948/c-catching-all-exceptions
        std::exception_ptr p = std::current_exception();
        m_logger_msg << "uncaught_exception in " << m_func_name << ": \n" <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
        logMessage(m_type, m_log_level, m_logger_msg, m_tag);
        return;
    }
    logMessage(m_type, m_log_level, "finish " + m_func_name, m_tag);

}
