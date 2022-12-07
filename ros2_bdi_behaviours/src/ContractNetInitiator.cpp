#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"
#include "ros2_bdi_behaviours/ConcreteContractNetInitiatorStates.hpp"

using namespace InitiatorStates;

ContractNetInitiator::ContractNetInitiator(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId) : ConversationsClient(desire_set, belief_set, agent_id, ConversationId), nAcceptances(0), nResponders(0)
{
	// All ContractNetInitiators are initially in SendCfp state
	currentState = &SendCfp::getInstance();
}

void ContractNetInitiator::setState(ContractNetInitiatorState& newState)
{
	currentState->exit(this);  // do stuff before we change state
	currentState = &newState;  // actually change states now
	currentState->entry(this); // do stuff after we change state
}

void ContractNetInitiator::react(ACLMessage const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}