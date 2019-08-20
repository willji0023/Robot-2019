#pragma once

#include <vector>
#include <string>

#include "communications/PublishNode.hpp"

class SubscriptionTree {
public:
    SubscriptionTree(std::string subtopic); 

    template <class P>
    void Publish(P p);

    void Subscribe(); 

    void Unsubscribe();

private:
    std::string m_subtopic;
    std::vector<PublishNode*> m_subList;
    std::vector<SubscriptionTree*> m_children;
};