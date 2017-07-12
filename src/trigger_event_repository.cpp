#include "trigger_event_repository.h"

#include <algorithm>
#include <iterator>
#include <utility>

#include <boost/circular_buffer.hpp>

namespace
{
template<class InputIterator, class OutputIterator, class Predicate>
inline OutputIterator copy_n_if(InputIterator first, InputIterator last, std::size_t n, OutputIterator result, Predicate pred)
{
    if(n > 0)
    {
        for(; n > 0 && first != last; ++first)
        {
            if(pred(*first))
            {
                *result = *first;
                ++result;
                --n;
            }
        }
    }
    return result;
}

inline bool satisfies(const ifttt::TriggerEvent & event, const ifttt::FieldConstraints & constraints)
{
    if(event.fields.size() != constraints.size())
    {
        return false;
    }
    return std::all_of(event.fields.begin(), event.fields.end(), [constraints](const ifttt::TriggerField & field)
    {
        try
        {
            return constraints.at(field.key_name) == field.value;
        }
        catch(const std::out_of_range &)
        {
            return false;
        }
    });
}
}

namespace ifttt
{
class TriggerEventRepository::Impl
{
    using Repository = boost::circular_buffer<TriggerEvent>;

public:
    Impl(std::size_t max_capacity) :
            repository_{max_capacity}
    {

    }

    void registerTriggerEvent(TriggerEvent event)
    {
        repository_.push_back(std::move(event));
    }

    std::vector<TriggerEvent> queryMatchingEvents(std::size_t max_num_matches, const FieldConstraints & field_constraints)
    {
        std::vector<TriggerEvent> results;
        results.reserve(max_num_matches);

        auto satisfies_constraints = std::bind(satisfies, std::placeholders::_1, field_constraints);
        copy_n_if(repository_.rbegin(), repository_.rend(), max_num_matches, std::back_inserter(results), satisfies_constraints);

        results.shrink_to_fit();
        return results;
    }
private:
    Repository repository_;
};


TriggerEventRepository::TriggerEventRepository(std::size_t max_capacity) :
        pimpl_{new Impl(max_capacity)}
{

}

TriggerEventRepository & TriggerEventRepository::operator=(TriggerEventRepository && other) = default;

TriggerEventRepository::~TriggerEventRepository() = default;

void TriggerEventRepository::registerTriggerEvent(TriggerEvent event)
{
    pimpl_->registerTriggerEvent(std::move(event));
}

std::vector<TriggerEvent> TriggerEventRepository::queryMatchingEvents(std::size_t max_num_matches, const FieldConstraints & field_constraints)
{
    return pimpl_->queryMatchingEvents(max_num_matches, field_constraints);
}
}