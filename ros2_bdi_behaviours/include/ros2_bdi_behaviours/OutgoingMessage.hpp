#pragma once
#include "ros2_bdi_behaviours/OutgoingMessageState.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

// Forward declaration to resolve circular dependency/include
class OutgoingMessageState;

// ----------------------------------------------------------------------------
// OutgoingMessage (FSM base class) declaration
//
class OutgoingMessage : public ConversationsClient
{
public:
	OutgoingMessage(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId);
	inline OutgoingMessageState* getCurrentState() const { return currentState; }
	void setState(OutgoingMessageState& newState);

	void receiveMsg(ACLMessage msg) override
  	{
    	this->react(msg);
  	}

	void react(ACLMessage const & event);

private:
	OutgoingMessageState* currentState;
};