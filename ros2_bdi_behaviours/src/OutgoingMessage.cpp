#include "ros2_bdi_behaviours/OutgoingMessage.hpp"
#include "ros2_bdi_behaviours/ConcreteOutgoingMessageStates.hpp"

using namespace OutgoingStates;

OutgoingMessage::OutgoingMessage(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId) : ConversationsClient(desire_set, belief_set, agent_id, ConversationId)
{
	// All OutgoingMessages are initially in SendCfp state
	currentState = &SendMessage::getInstance();
}

void OutgoingMessage::setState(OutgoingMessageState& newState)
{
	currentState->exit(this);  // do stuff before we change state
	currentState = &newState;  // actually change states now
	currentState->entry(this); // do stuff after we change state
}

void OutgoingMessage::react(ACLMessage const & event)
{
	// Delegate the task of determining the next state to the current state
	currentState->react(this, event);
}