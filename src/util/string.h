#ifndef __STRING_H__
#define __STRING_H__

#include <string>

bool endsWith(std::string const& fullString, std::string const& ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare(fullString.length() - ending.length(),
                                    ending.length(), ending));
  } else {
    return false;
  }
}

bool endsWith(std::string const& fullString, const char* ending) {
  size_t length = strlen(ending);
  if (fullString.length() >= length) {
    return (0 ==
            fullString.compare(fullString.length() - length, length, ending));
  } else {
    return false;
  }
}

#endif
