#include "ros2_bdi_behaviours/ContractNetResponder.hpp"
#include "ros2_bdi_behaviours/ConcreteContractNetResponderStates.hpp"

using namespace ResponderStates;

ContractNetResponder::ContractNetResponder(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId) : ConversationsClient(desire_set, belief_set, agent_id, ConversationId)
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

void ContractNetResponder::react(ACLMessage const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}

void ContractNetResponder::react(ActionOver const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}