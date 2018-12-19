/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (MRBSP),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
  * @file: function_logger.h
  * @brief: defines classes and macros for creating one-line log commands in functions.
  * @author: Nikita Dizhur
  *
  */

#ifndef FUNCTION_LOGGER_H
#define FUNCTION_LOGGER_H


#include <string>
#include <sstream>
#include <boost/filesystem.hpp>

namespace MRBSP {

    namespace Utils {

        /**
         * log types
         */
        enum LogType {
            info,
            warn,
            error,
            critical,
        };

        enum class LogTag {
            all,
            utils,
            logger,
            odometry,
            da,
        };

        // taken from: http://stackoverflow.com/questions/1082192/how-to-generate-random-variable-names-in-c-using-macros:
        // This is some crazy magic that helps produce __BASE__247
        // Vanilla interpolation of __BASE__##__LINE__ would produce __BASE____LINE__
        // I still can't figure out why it works, but it has to do with macro resolution ordering
        #define _PP_CAT(a, b) _PP_CAT_I(a, b)
        #define _PP_CAT_I(a, b) _PP_CAT_II(~, a ## b)
        #define _PP_CAT_II(p, res) res

        #define _UNIQUE_NAME(base) _PP_CAT(base, __COUNTER__)

        #define _LOG_ADD(type, log_level, name, tag) FunctionLogger _UNIQUE_NAME(logger)(type, log_level, name, tag)

        #define FUNCTION_LOGGER(tag) _LOG_ADD(LogType::info, 2, BOOST_CURRENT_FUNCTION, tag)


        class FunctionLogger {
        public:
            FunctionLogger(LogType type, int log_level, const std::string &func_name, LogTag tag);

            ~FunctionLogger();

        protected:
            const LogType m_type;
            const int m_log_level;
            const std::string m_func_name;
            const LogTag m_tag;
            std::stringstream m_logger_msg;
        };
    }
}
#endif //FUNCTION_LOGGER_H
