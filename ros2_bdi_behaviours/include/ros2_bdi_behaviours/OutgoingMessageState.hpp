#pragma once
#include "ros2_bdi_behaviours/OutgoingMessage.hpp"

// Forward declaration to resolve circular dependency/include
class OutgoingMessage;
class ACLMessage;

class OutgoingMessageState
{
public:
	virtual void entry(OutgoingMessage* outgoingMessage) = 0;
	virtual void react(OutgoingMessage* outgoingMessage, ACLMessage const & Message) = 0;
	virtual void exit(OutgoingMessage* outgoingMessage) = 0;
	virtual ~OutgoingMessageState() {}
};