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
struct MsgReceived{ ACLMessage msg; };
struct ActionOver{};

// ----------------------------------------------------------------------------
// ContractNetResponder (FSM base class) declaration
//
class ContractNetResponder : public ConversationsClient
{
public:
	ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set);
	inline ContractNetResponderState* getCurrentState() const { return currentState; }
	void setState(ContractNetResponderState& newState);

	void receiveMsg(ACLMessage msg) override
  	{
		inbox.push_back(msg);
    	MsgReceived Message;
    	Message.msg = msg;
    	react(Message);
  	}

	void react(MsgReceived const & event);
	void react(ActionOver const & event);

	virtual ACLMessage handleCfp(ACLMessage cfp) { return ACLMessage();};
	virtual ACLMessage handleAcceptProposal(ACLMessage cfp, ACLMessage propose,ACLMessage accept) { return ACLMessage();};
	virtual void handleRejectProposal(ACLMessage cfp, ACLMessage propose, ACLMessage reject) { };

	ACLMessage cfp, propose, accept, inform, reject;
	std::vector<ACLMessage> inbox;

private:
	ContractNetResponderState* currentState;
};