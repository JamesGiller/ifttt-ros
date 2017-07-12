#include "trigger_event_repository.h"

#include <utility>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace ifttt;

static TriggerField generate_type_field(std::string value)
{
    TriggerField type_field;
    type_field.key_name = "type";
    type_field.value = std::move(value);    
    return type_field;
}

static TriggerEvent generate_test_event(std::string type_field_value, float seconds_in_past)
{
    TriggerEvent event;

    event.header.stamp = ros::Time::now();
    if(seconds_in_past > 0.0)
    {
        event.header.stamp -= ros::Duration(seconds_in_past);
    }

    auto type_field = generate_type_field(std::move(type_field_value));

    event.fields.emplace_back(std::move(type_field));

    return event;
}

class TriggerEventRepositoryTest : public testing::Test
{
public:
    TriggerEventRepositoryTest() :
            repository(TEST_REPOSITORY_CAPACITY)
    {
        
    }
protected:
    static constexpr std::size_t TEST_REPOSITORY_CAPACITY = 2;

    TriggerEventRepository repository;
};

TEST_F(TriggerEventRepositoryTest, QueryMatchingEventsReturnsMatch)
{
    auto type_A_trigger_event = generate_test_event("A", 0.0);

    repository.registerTriggerEvent(type_A_trigger_event);

    FieldConstraints constraints;
    constraints["type"] = "A";

    auto matching_events = repository.queryMatchingEvents(TEST_REPOSITORY_CAPACITY, constraints);
    ASSERT_EQ(1, matching_events.size());
}

TEST_F(TriggerEventRepositoryTest, QueryMatchingEventsObeysReturnLimit)
{
    auto early_type_A_trigger_event = generate_test_event("A", 5.0);

    repository.registerTriggerEvent(early_type_A_trigger_event);

    auto latest_type_A_trigger_event = generate_test_event("A", 0.0);

    repository.registerTriggerEvent(latest_type_A_trigger_event);

    FieldConstraints constraints;
    constraints["type"] = "A";

    auto matching_events = repository.queryMatchingEvents(1, constraints);
    ASSERT_EQ(1, matching_events.size());
    ASSERT_EQ(latest_type_A_trigger_event.header.stamp, matching_events[0].header.stamp);
}

TEST_F(TriggerEventRepositoryTest, QueryMatchingEventsReturnsNoMatch)
{
    auto type_A_trigger_event = generate_test_event("A", 0.0);

    repository.registerTriggerEvent(type_A_trigger_event);

    FieldConstraints constraints;
    constraints["type"] = "Unmatched";

    auto matching_events = repository.queryMatchingEvents(TEST_REPOSITORY_CAPACITY, constraints);
    ASSERT_EQ(0, matching_events.size()) << "matching event type: " << matching_events[0].fields[0].value;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}