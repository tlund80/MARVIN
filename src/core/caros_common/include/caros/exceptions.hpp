#ifndef CAROS_EXCEPTIONS_HPP__
#define CAROS_EXCEPTIONS_HPP__

#include <stdexcept>
#include <string>
#include <sstream>

/*
 * @brief Throw an unavailableService exception with the message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_UNAVAILABLE_SERVICE("The service " << serviceName << " is unavailable.");
 * \endcode
 */
#define THROW_CAROS_UNAVAILABLE_SERVICE(ostreamExpression) do { \
    std::ostringstream CAROS__message; \
    CAROS__message << ostreamExpression; \
    throw caros::unavailableService(CAROS__message.str()); \
    } while (0)

/*
 * @brief Throw a badServiceCall exception with the message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_BAD_SERVICE_CALL("An unexpected error happened while calling the service " << serviceName);
 * \endcode
 */
#define THROW_CAROS_BAD_SERVICE_CALL(ostreamExpression) do { \
    std::ostringstream CAROS__message; \
    CAROS__message << ostreamExpression; \
    throw caros::badServiceCall(CAROS__message.str()); \
    } while (0)

namespace caros {
    /**
     * @brief unavailableService exception.
     *
     * TODO: How, what and why
     */
    class unavailableService : public std::runtime_error {
        public:
        explicit unavailableService(const std::string& what) : runtime_error(what) {
            /* Empty */
        }

        virtual ~unavailableService() throw() {
            /* Empty */
        }
    };

    /**
     * @brief badServiceCall exception.
     *
     * TODO: How, what and why
     */
    class badServiceCall : public std::runtime_error {
        public:
        explicit badServiceCall(const std::string& what) : runtime_error(what) {
            /* Empty */
        }
        virtual ~badServiceCall() throw() {
            /* Empty */
        }
    };
}

#endif /* CAROS_EXCEPTIONS_HPP__ */
