#ifndef IFTTT_TRIGGER_EVENT_REPOSITORY_H
#define IFTTT_TRIGGER_EVENT_REPOSITORY_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ifttt/TriggerEvent.h"

namespace ifttt
{
using FieldConstraints = std::unordered_map<std::string, std::string>;

class TriggerEventRepository
{
public:
  explicit TriggerEventRepository(std::size_t max_capacity);

  TriggerEventRepository(TriggerEventRepository &&);

  TriggerEventRepository(const TriggerEventRepository &) = delete;

  TriggerEventRepository & operator=(TriggerEventRepository &&);

  TriggerEventRepository & operator=(const TriggerEventRepository &) = delete;

  ~TriggerEventRepository();

  void registerTriggerEvent(TriggerEvent event);

  std::vector<TriggerEvent> queryMatchingEvents(std::size_t max_num_matches,
                                                const FieldConstraints & field_constraints) const;

private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
}
#endif