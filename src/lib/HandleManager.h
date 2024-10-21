#ifndef _HANDLE_MANAGER_H_
#define _HANDLE_MANAGER_H_

#include <map>

template<typename PointerType, typename HandleType>
class HandleManager {

public:
  HandleManager() {}

  HandleType create(PointerType p) {
    HandleType handle = static_cast<HandleType>(_counter++);
    _handles[handle] = p;
    return handle;
  }

  PointerType lookup(HandleType handle) {
    auto it = _handles.find(handle);
    if(it == _handles.end()) {
      return nullptr;
    }
    return it->second;
  }

  bool remove(HandleType handle) {
    auto it = _handles.find(handle);
    if(it == _handles.end()) {
      return false;
    }
    _handles.erase(it);
    return true;
  }

private:
  HandleType _counter = 1;
  std::map<HandleType, PointerType> _handles;
};

#endif
