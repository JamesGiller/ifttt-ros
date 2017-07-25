#include "trigger_event_repository.h"

#include <utility>

#include <ros/ros.h>
#include <gtest/gtest.h>

using namespace ifttt;

TriggerField generateTriggerField(std::string key_name, std::string value);
TriggerEvent generateEventForTest(std::string field_key_name, std::string field_value, float seconds_in_past);

struct TriggerEventRepositoryTest : public testing::Test
{
  static const std::size_t repository_size_for_test;
  static const std::size_t size_to_test_query_return_limit;

  TriggerEventRepositoryTest() : repository(repository_size_for_test)
  {
  }

  TriggerEventRepository repository;
};

const std::size_t TriggerEventRepositoryTest::repository_size_for_test = 2;
const std::size_t TriggerEventRepositoryTest::size_to_test_query_return_limit = 1;

TEST_F(TriggerEventRepositoryTest, QueryMatchingEvents_ShouldReturnMatch)
{
  auto trigger_event = generateEventForTest("type", "A", 0.);

  FieldConstraints constraints;
  constraints["type"] = "A";

  auto matching_events = repository.queryMatchingEvents(repository_size_for_test, constraints);
  ASSERT_EQ(0, matching_events.size());

  repository.registerTriggerEvent(trigger_event);

  matching_events = repository.queryMatchingEvents(repository_size_for_test, constraints);
  ASSERT_EQ(1, matching_events.size());
}

TEST_F(TriggerEventRepositoryTest, QueryMatchingEvents_ObeysReturnLimit)
{
  auto earlier_trigger_event = generateEventForTest("type", "A", 5.);

  repository.registerTriggerEvent(earlier_trigger_event);

  auto latest_trigger_event = generateEventForTest("type", "A", 0.);

  repository.registerTriggerEvent(latest_trigger_event);

  FieldConstraints constraints;
  constraints["type"] = "A";

  auto matching_events = repository.queryMatchingEvents(size_to_test_query_return_limit, constraints);
  ASSERT_EQ(size_to_test_query_return_limit, matching_events.size());
  ASSERT_EQ(latest_trigger_event.header.stamp, matching_events[0].header.stamp);
}

TEST_F(TriggerEventRepositoryTest, QueryMatchingEvents_ShouldReturnNoMatch)
{
  auto trigger_event = generateEventForTest("type", "A", 0.);

  FieldConstraints constraints;
  constraints["type"] = "Unmatched";

  auto matching_events = repository.queryMatchingEvents(repository_size_for_test, constraints);
  ASSERT_EQ(0, matching_events.size()) << matching_events[0].fields[0].value << " should not match "
                                       << constraints["type"];
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TriggerField generateTriggerField(std::string key_name, std::string value)
{
  TriggerField field;
  field.key_name = key_name;
  field.value = std::move(value);
  return field;
}

TriggerEvent generateEventForTest(std::string field_key_name, std::string field_value, float seconds_in_past)
{
  TriggerEvent event;

  event.header.stamp = ros::Time::now();
  if (seconds_in_past > 0.0)
  {
    event.header.stamp -= ros::Duration(seconds_in_past);
  }

  auto field = generateTriggerField(std::move(field_key_name), std::move(field_value));

  event.fields.emplace_back(std::move(field));

  return event;
}
