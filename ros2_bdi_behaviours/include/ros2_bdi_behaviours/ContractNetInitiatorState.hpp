#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"

// Forward declaration to resolve circular dependency/include
class ContractNetInitiator;
class ACLMessage;

class ContractNetInitiatorState
{
public:
	virtual void entry(ContractNetInitiator* contractNetInitiator) = 0;
	virtual void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message) = 0;
	virtual void exit(ContractNetInitiator* contractNetInitiator) = 0;
	virtual ~ContractNetInitiatorState() {}
};