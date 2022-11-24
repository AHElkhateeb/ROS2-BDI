#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

class OrganizerBehaviour : public ContractNetInitiator
{
public:
	OrganizerBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id);

	void handlePropose(ACLMessage propose);
	void handleReject(ACLMessage reject);
	void handleInform(ACLMessage inform);
	void handleAllResultNotifications(std::vector<ACLMessage> resultNotifications);
	std::vector<ACLMessage> handleAllResponses(std::vector<ACLMessage> responses);
};