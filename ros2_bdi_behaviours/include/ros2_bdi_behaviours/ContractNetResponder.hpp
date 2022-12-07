#pragma once
#include "ros2_bdi_behaviours/ContractNetResponderState.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

// Forward declaration to resolve circular dependency/include
class ContractNetResponderState;

// ----------------------------------------------------------------------------
// Event declarations
//
struct ActionOver{};

// ----------------------------------------------------------------------------
// ContractNetResponder (FSM base class) declaration
//
class ContractNetResponder : public ConversationsClient
{
public:
	ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId);
	inline ContractNetResponderState* getCurrentState() const { return currentState; }
	void setState(ContractNetResponderState& newState);

	void receiveMsg(ACLMessage msg) override
  	{
		inbox.push_back(msg);
    	this->react(msg);
  	}

	void react(ACLMessage const & event);
	void react(ActionOver const & event);

	virtual ACLMessage handleCfp(ACLMessage cfp) = 0;
	virtual ACLMessage handleAcceptProposal(ACLMessage cfp, ACLMessage propose, ACLMessage accept) = 0;
	virtual void handleRejectProposal(ACLMessage cfp, ACLMessage propose, ACLMessage reject) { };

	ACLMessage cfp, propose, accept, inform, reject;
	std::vector<ACLMessage> inbox;

private:
	ContractNetResponderState* currentState;
};