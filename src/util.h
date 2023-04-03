#ifndef UTIL_H_
#define UTIL_H_

#include <string>
#include <vector>

namespace ppr {

std::vector<std::string> split(const std::string &str,
                               const std::string &delim);

} // namespace ppr

#endif /* UTIL_H_ */
