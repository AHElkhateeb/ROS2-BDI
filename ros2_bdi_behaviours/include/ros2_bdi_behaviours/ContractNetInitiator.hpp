#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiatorState.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/conversations_client.hpp"

using ACLConversations::ConversationsClient;

// Forward declaration to resolve circular dependency/include
class ContractNetInitiatorState;

// ----------------------------------------------------------------------------
// Event declarations
//
struct MsgReceived{ ACLMessage msg; };

// ----------------------------------------------------------------------------
// ContractNetInitiator (FSM base class) declaration
//
class ContractNetInitiator : public ConversationsClient
{
public:
	ContractNetInitiator(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set);
	inline ContractNetInitiatorState* getCurrentState() const { return currentState; }
	void setState(ContractNetInitiatorState& newState);

	void receiveMsg(ACLMessage msg) override
  	{
    	MsgReceived Message;
    	Message.msg = msg;
    	react(Message);
  	}

	void react(MsgReceived const & event);

	//Probably will be kept empty in current implementation
	virtual void handlePropose(ACLMessage propose) { };
	virtual void handleReject(ACLMessage reject) { };
	virtual void handleNotUnderstood(ACLMessage notUnderstood) { };
	
	virtual void handleFailure(ACLMessage failure) { };
	virtual std::vector<ACLMessage> handleAllResponses(std::vector<ACLMessage> responses) { return {ACLMessage("ACCEPT")}; };
	virtual void handleInform(ACLMessage inform) { };
	virtual void handleAllResultNotifications(std::vector<ACLMessage> resultNotifications) { }; //This method is called when all the result notification messages have been collected.
	virtual void handleAllRejected() { };

	int nResponders, nAcceptances;
	ACLMessage cfp;
	std::vector<ACLMessage> informs; //All inform received messages
	std::vector<ACLMessage> responses; //All the <code>PROPOSE, REFUSE, NOT-UNDERSTOOD</code> received messages 
	std::vector<ACLMessage> acceptances; //the list of ACCEPT/REJECT_PROPOSAL to be sent back.
	std::vector<ACLMessage> resultNotifications; //All the <code>inform, failure</code> received messages, which are not not out-of-sequence according to the protocol rules.

private:
	ContractNetInitiatorState* currentState;
};