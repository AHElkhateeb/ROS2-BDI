#include "ContractNetResponder.hpp"
#include "ConcreteContractNetResponderStates.hpp"

ContractNetResponder::ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set) : ConversationsClient(desire_set, belief_set)
{
	// All ContractNetResponders are initially in ReceiveCfp state
	currentState = &ReceiveCfp::getInstance();
}

void ContractNetResponder::setState(ContractNetResponderState& newState)
{
	currentState->exit(this);  // do stuff before we change state
	currentState = &newState;  // actually change states now
	currentState->entry(this); // do stuff after we change state
}

void ContractNetResponder::react(MsgReceived const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}

void ContractNetResponder::react(ActionOver const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}