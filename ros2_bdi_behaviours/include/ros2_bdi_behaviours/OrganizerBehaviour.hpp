#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

class OrganizerBehaviour : public ContractNetInitiator
{
public:
	OrganizerBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id);
};