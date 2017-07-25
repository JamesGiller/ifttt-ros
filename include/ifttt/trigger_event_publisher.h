#ifndef IFTTT_TRIGGER_EVENT_PUBLISHER_H
#define IFTTT_TRIGGER_EVENT_PUBLISHER_H

#include <functional>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "ifttt/TriggerEvent.h"

namespace ifttt
{
template <typename SrcMsg>
using TriggerEventEncoder = std::function<TriggerEvent(const SrcMsg &)>;

template <typename SrcMsg>
class TriggerEventPublisher
{
public:
  TriggerEventPublisher(std::string src_topic_name, uint32_t src_topic_queue_size,
                        TriggerEventEncoder<SrcMsg> to_trigger_event, std::string event_topic_name,
                        uint32_t event_topic_queue_size)
      : nh_(), to_trigger_event_(std::move(to_trigger_event))
  {
    event_publisher_ = nh_.advertise<TriggerEvent>(event_topic_name, event_topic_queue_size);

    src_topic_subscriber_ =
        nh_.subscribe(src_topic_name, src_topic_queue_size, &TriggerEventPublisher<SrcMsg>::srcTopicCallback_, this);
  }

private:
  ros::NodeHandle nh_;
  TriggerEventEncoder<SrcMsg> to_trigger_event_;
  ros::Publisher event_publisher_;
  ros::Subscriber src_topic_subscriber_;

  void srcTopicCallback_(const typename SrcMsg::ConstPtr &msg) const
  {
    auto event = to_trigger_event_(*msg);
    event_publisher_.publish(event);
  }
};
}
#endif