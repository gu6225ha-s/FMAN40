#ifndef STR_UTIL_H_
#define STR_UTIL_H_

#include <string>
#include <vector>

namespace ppr {

std::vector<std::string> split(const std::string &str,
                               const std::string &delim);

} // namespace ppr

#endif /* STR_UTIL_H_ */
