#define ValueOrReturn(name, function, return_value) \
  auto maybe_##name = function; \
  if(!maybe_##name.has_value()) { \
    std::cout << "Could not find value" << std::endl; \
    return return_value; \
  } \
  auto name = maybe_##name.value(); \

#define ReturnOnFalse(function, return_value) \
  if(!function) { \
    return return_value; \
  } 
